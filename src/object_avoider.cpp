#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

enum State
{
    DRIVING_RIGHT,
    SWITCHING_LEFT,
    DRIVING_LEFT,
    SWITCHING_RIGHT
};

class ObjectAvoider : public rclcpp::Node
{
public:
    ObjectAvoider() : Node("object_avoider")
    {
        // Paraméterek deklarálása
        this->declare_parameter("drive_speed", 1.0);      // Előre haladás sebessége (m/s)
        this->declare_parameter("kp", 2.0);               // PID arányos erősítés

        // Parancsadó - sebességvetület (twist) publikálása a járműnek
        publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // QoS beállítások az előfizetésekhez
        rclcpp::QoS qos(10);
        qos.keep_last(10);
        qos.best_effort();
        qos.durability_volatile();

        // Előfizetések az érzékelőkből
        // 1. Elülső lézeres szenzor - akadálya detekció
        sub_scan_front_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&ObjectAvoider::front_scan_callback, this, std::placeholders::_1));

        // 2. Bal cliff szenzor - szakadék detekció a bal oldalon
        sub_cliff_left_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/cliff_left", qos, std::bind(&ObjectAvoider::left_cliff_callback, this, std::placeholders::_1));

        // 3. Jobb cliff szenzor - szakadék detekció a jobb oldalon
        sub_cliff_right_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/cliff_right", qos, std::bind(&ObjectAvoider::right_cliff_callback, this, std::placeholders::_1));

        // 4. Odometria - autó helyzete és orientációja
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos, std::bind(&ObjectAvoider::odom_callback, this, std::placeholders::_1));

        // Kezdeti állapot: a jobb sávban haladunk
        state_ = DRIVING_RIGHT;
        target_lane_y_ = -2.0;

        // Cliff korrekció változóinak inicializálása
        is_correcting_ = false;
        correction_start_time_ = this->now();
        correction_turn_val_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "🚗 ObjectAvoider csomópont elindult. Akadálykerülés és cliff-detekció aktív!");
        RCLCPP_INFO(this->get_logger(), "   Cliff korrekció időtartama: 1.0 másodperc");
    }

private:
    // Odometria callback - frissíti az autó aktuális pozícióját és orientációját
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Y pozíció (oldalirányú eltolódás)
        current_y_ = msg->pose.pose.position.y;

        // Quaternion konvertálás Euler szögre (csak yaw-ra van szükségünk)
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        // Vezérlési logika végrehajtása
        control_logic();
    }

    // Elülső lézeres szenzor feldolgozása - akadályt detektál az elülső 30 fokos szögben
    void front_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        bool detected = false;
        int center = msg->ranges.size() / 2;
        int width = 15;  // +/- 15 sugár = ~30 fok
        
        for (int i = center - width; i < center + width; ++i)
        {
            if (msg->ranges[i] > 0.1 && msg->ranges[i] < 4.0)
            {
                detected = true;
                break;
            }
        }
        
        // Ha az állapot változott, kiírjuk a konzolra
        if (detected && !obstacle_front_)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️  AKADÁLY ÉRZÉKELVE ELÖL! (távolság: ~4m alatt)");
        }
        else if (!detected && obstacle_front_)
        {
            RCLCPP_INFO(this->get_logger(), "✓ Akadály elhárítva, folytattuk az utat.");
        }
        
        obstacle_front_ = detected;
    }

    // Bal oldali szakadék/cliff szenzor - detektálja ha nincs támasz a bal oldalon
    void left_cliff_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.size() > 0)
        {
            bool detected = (msg->ranges[0] > 1.0);
            
            if (detected && !cliff_left_detected_)
            {
                RCLCPP_WARN(this->get_logger(), "🔴 SZAKADÉK ÉRZÉKELVE BAL OLDALON!");
            }
            else if (!detected && cliff_left_detected_)
            {
                RCLCPP_INFO(this->get_logger(), "✓ Bal oldal biztonságos.");
            }
            
            cliff_left_detected_ = detected;
        }
    }

    // Jobb oldali szakadék/cliff szenzor - detektálja ha nincs támasz a jobb oldalon
    void right_cliff_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (msg->ranges.size() > 0)
        {
            bool detected = (msg->ranges[0] > 1.0);
            
            if (detected && !cliff_right_detected_)
            {
                RCLCPP_WARN(this->get_logger(), "🔴 SZAKADÉK ÉRZÉKELVE JOBB OLDALON!");
            }
            else if (!detected && cliff_right_detected_)
            {
                RCLCPP_INFO(this->get_logger(), "✓ Jobb oldal biztonságos.");
            }
            
            cliff_right_detected_ = detected;
        }
    }

    // Vezérlési logika - управляет движением автомобиля на основе датчиков
    void control_logic()
    {
        rclcpp::Time now = this->now();
        auto msg = geometry_msgs::msg::Twist();

        // --- 1. CLIFF KORREKCIÓ (Trigger) ---
        // Ha érzékeljük a cliff-et, és épp nem korrigálunk, elindítjuk az időzítőt
        if (!is_correcting_)
        {
            if (cliff_left_detected_)
            {
                // Bal cliff -> Jobbra korrigálunk (1 másodpercig)
                is_correcting_ = true;
                correction_start_time_ = now;
                correction_turn_val_ = -0.4; // Jobbra fordulás
                RCLCPP_WARN(this->get_logger(), "🚗 BALRA IRÁNYÍTOTT KORREKCIÓ! (1.0s) - Szakadék balra van!");
            }
            else if (cliff_right_detected_)
            {
                // Jobb cliff -> Balra korrigálunk (1 másodpercig)
                is_correcting_ = true;
                correction_start_time_ = now;
                correction_turn_val_ = 0.4; // Balra fordulás
                RCLCPP_WARN(this->get_logger(), "🚗 JOBBRA IRÁNYÍTOTT KORREKCIÓ! (1.0s) - Szakadék jobbra van!");
            }
        }

        // --- 2. KORREKCIÓ VÉGREHAJTÁSA (Timer) ---
        if (is_correcting_)
        {
            double elapsed = (now - correction_start_time_).seconds();

            // 1.0 másodpercig tartjuk a korrekciót
            if (elapsed < 1.0)
            {
                msg.linear.x = 0.6; // Kicsit lassítunk a stabilitásért
                msg.angular.z = correction_turn_val_;
                publisher_cmd_vel_->publish(msg);

                // Míg korrigálunk, nem futtatjuk a PID-et, hogy ne zavarjon be
                return;
            }
            else
            {
                // Idő letelt, visszatérünk a normál vezetéshez
                is_correcting_ = false;
                RCLCPP_INFO(this->get_logger(), "✓ Cliff korrekció kész. Normál vezetés folytatódik.");
            }
        }

        // --- 3. NORMÁL VEZETÉS (PID + Állapotgép) ---

        // Állapotgép (State Machine) - akadálykerüléshez
        switch (state_)
        {
        case DRIVING_RIGHT:
            // Alapértelmezett állapot: jobboldali sávban haladunk
            target_lane_y_ = -2.0;
            if (obstacle_front_)
            {
                RCLCPP_WARN(this->get_logger(), "🛑 AKADÁLY ELÖL! Balra kitérünk...");
                state_ = SWITCHING_LEFT;
            }
            break;
            
        case SWITCHING_LEFT:
            // Átmenet a jobb sávból a bal sávba
            target_lane_y_ = 2.0;
            if (current_y_ > 1.5)
            {
                RCLCPP_INFO(this->get_logger(), "✓ Bal sávba érkeztünk. Folytatódik a vezetés.");
                state_ = DRIVING_LEFT;
            }
            break;
            
        case DRIVING_LEFT:
            // Bal sávban vezetés az akadály megkerülése után
            target_lane_y_ = 2.0;
            if (obstacle_front_)
            {
                RCLCPP_WARN(this->get_logger(), "🛑 MÁSIK AKADÁLY ELÖL! Jobbra kitérünk...");
                state_ = SWITCHING_RIGHT;
            }
            break;
            
        case SWITCHING_RIGHT:
            // Átmenet a bal sávból a jobb sávba
            target_lane_y_ = -2.0;
            if (current_y_ < -1.5)
            {
                RCLCPP_INFO(this->get_logger(), "✓ Jobb sávba érkeztünk. Normál vezetés folytatódik.");
                state_ = DRIVING_RIGHT;
            }
            break;
        }

        // PID Szabályozó - az autó pozícióját az elérni kívánt sáv felé állítja be
        double speed = this->get_parameter("drive_speed").as_double();
        double kp = this->get_parameter("kp").as_double();

        double lookahead = (state_ == DRIVING_RIGHT || state_ == DRIVING_LEFT) ? 4.0 : 2.5;
        double error_y = target_lane_y_ - current_y_;
        double desired_heading = std::atan2(error_y, lookahead);
        double heading_error = desired_heading - current_yaw_;

        // Szögek normalizálása a (-π, π] tartományra
        while (heading_error > M_PI)
            heading_error -= 2 * M_PI;
        while (heading_error < -M_PI)
            heading_error += 2 * M_PI;

        double angular_z = kp * heading_error;

        // Szögsebességet limitáljuk az utazás stabilitása érdekében
        if (angular_z > 1.0)
            angular_z = 1.0;
        if (angular_z < -1.0)
            angular_z = -1.0;

        msg.linear.x = speed;
        msg.angular.z = angular_z;

        publisher_cmd_vel_->publish(msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_front_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_cliff_left_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_cliff_right_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double target_lane_y_ = -2.0;
    bool obstacle_front_ = false;

    bool cliff_left_detected_ = false;
    bool cliff_right_detected_ = false;

    bool is_correcting_;
    rclcpp::Time correction_start_time_;
    double correction_turn_val_;

    State state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectAvoider>());
    rclcpp::shutdown();
    return 0;
}