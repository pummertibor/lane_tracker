#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

using namespace std::chrono_literals;

// Állapotok definíciója a szigorú manőverezéshez
enum State {
    DRIVING_RIGHT,      // Normál haladás a jobb sávban
    SWITCHING_LEFT,     // Épp váltunk balra (kötelező elérni a sávot)
    DRIVING_LEFT,       // Bal sávban haladunk (várjuk, hogy elhagyjuk az akadályt)
    SWITCHING_RIGHT     // Épp váltunk vissza jobbra
};

class ObjectAvoider : public rclcpp::Node {
public:
    ObjectAvoider() : Node("object_avoider") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        rclcpp::QoS qos(10);
        qos.keep_last(10);
        qos.best_effort();
        qos.durability_volatile();

        subscription_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&ObjectAvoider::scan_callback, this, std::placeholders::_1));

        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos, std::bind(&ObjectAvoider::odom_callback, this, std::placeholders::_1));

        // Kezdőállapot
        state_ = DRIVING_RIGHT;
        target_lane_y_ = -2.0; 
        
        RCLCPP_INFO(this->get_logger(), "Rendszer indul. Allapot: DRIVING_RIGHT");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_y_ = msg->pose.pose.position.y;
        
        // Orientáció (Yaw)
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        control_logic();
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Akadálykeresés
        bool obstacle_ahead = false;
        int center_idx = msg->ranges.size() / 2;
        int scan_width = 45; // Széles látószög
        
        float min_dist = 100.0;

        for (int i = center_idx - scan_width; i < center_idx + scan_width; ++i) {
            if (i >= 0 && i < (int)msg->ranges.size()) {
                float d = msg->ranges[i];
                if (d > 0.1 && d < 3.5) {
                    obstacle_ahead = true;
                    if (d < min_dist) min_dist = d;
                }
            }
        }

        // Frissítjük a változót, amit a control_logic használ
        obstacle_detected_ = obstacle_ahead;
    }

    void control_logic() {
        rclcpp::Time now = this->now();

        // --- ÁLLAPOTGÉP (STATE MACHINE) ---
        switch (state_) {
            case DRIVING_RIGHT:
                target_lane_y_ = -2.0;
                // Ha akadály van, azonnal váltunk
                if (obstacle_detected_) {
                    RCLCPP_WARN(this->get_logger(), "AKADALY! -> Savvaltas inditasa (SWITCHING_LEFT)");
                    state_ = SWITCHING_LEFT;
                }
                break;

            case SWITCHING_LEFT:
                target_lane_y_ = 2.0;
                // Ellenőrzés: Megérkeztünk-e TELJESEN a bal sávba?
                // A bal sáv közepe 2.0. Ha 1.8 fölött vagyunk, akkor már ott vagyunk.
                if (current_y_ > 1.8) {
                    RCLCPP_INFO(this->get_logger(), "Bal sav elerve. Varunk a biztonsagos elhaladasra (DRIVING_LEFT)...");
                    state_ = DRIVING_LEFT;
                    left_lane_arrival_time_ = now; // Időzítő indítása
                }
                break;

            case DRIVING_LEFT:
                target_lane_y_ = 2.0;
                // Két feltétel a visszatéréshez:
                // 1. Nincs akadály
                // 2. Eltelt legalább 4 másodperc azóta, hogy átértünk (biztonsági tartalék)
                {
                    double time_in_lane = (now - left_lane_arrival_time_).seconds();
                    
                    if (!obstacle_detected_ && time_in_lane > 4.0) {
                        RCLCPP_INFO(this->get_logger(), "Akadaly kikerulve -> Visszateres (SWITCHING_RIGHT)");
                        state_ = SWITCHING_RIGHT;
                    }
                }
                break;

            case SWITCHING_RIGHT:
                target_lane_y_ = -2.0;
                // Ellenőrzés: Megérkeztünk-e vissza?
                if (current_y_ < -1.8) {
                    RCLCPP_INFO(this->get_logger(), "Visszaertunk a jobb savba (DRIVING_RIGHT).");
                    state_ = DRIVING_RIGHT;
                }
                break;
        }

        // --- MOZGÁS VEZÉRLÉS ---
        auto msg = geometry_msgs::msg::Twist();
        
        double error_y = target_lane_y_ - current_y_;
        
        // Dinamikus Lookahead: Ha sávot váltunk, agresszívebben kormányzunk
        double lookahead = 3.5; 
        if (state_ == SWITCHING_LEFT || state_ == SWITCHING_RIGHT) {
            lookahead = 2.5; // Kisebb lookahead = élesebb kanyarodás a cél sáv felé
        }

        double desired_heading = std::atan2(error_y, lookahead);
        double heading_error = desired_heading - current_yaw_;

        msg.angular.z = 1.5 * heading_error;
        msg.linear.x = 1.0; 

        publisher_->publish(msg);
    }

    // Változók
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;

    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    double target_lane_y_ = -2.0;
    
    bool obstacle_detected_ = false;
    State state_; 
    rclcpp::Time left_lane_arrival_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectAvoider>());
    rclcpp::shutdown();
    return 0;
}