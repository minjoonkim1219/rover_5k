#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

class MovingRover : public rclcpp::Node {

    public:
      MovingRover() : Node("MovingRover") {
        pub = this->create_publisher<geometry_msgs::msg::Twist>("/rover_5k/cmd_vel", 10);
        timer = this->create_wall_timer(500ms, std::bind(&MovingRover::timer_callback, this));
      }

    private:
      void timer_callback() {
        auto command = geometry_msgs::msg::Twist();
        command.linear.x = 0.3;                        // 주기적으로(500ms) 직진 메시지를 퍼블리시 한다.
        command.angular.z = 0;
        RCLCPP_INFO(this->get_logger(), "Moving command [x: '%.2f', z: '%.2f']",
                    command.linear.x, command.angular.z);
        pub->publish(command);
      }

      rclcpp::TimerBase::SharedPtr timer;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;

 };

 int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<MovingRover>();
   RCLCPP_INFO(node->get_logger(), "Start moving Rover!");
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
 }
