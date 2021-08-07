#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

class ClimbCrater : public rclcpp::Node {

    public:
      ClimbCrater() : Node("ClimbCrater"), flag(0) {
        pub = this->create_publisher<geometry_msgs::msg::Twist>("/rover_5k/cmd_vel", 10);
        sub = this->create_subscription<nav_msgs::msg::Odometry>("/rover_5k/odom", 10,
            std::bind(&ClimbCrater::climbcrater_callback, this, _1));
      }

    private:

      void climbcrater_callback(nav_msgs::msg::Odometry::SharedPtr msg) {

        auto command = this->goCrater(msg);

        pub->publish(command);
      }

      geometry_msgs::msg::Twist goCrater(nav_msgs::msg::Odometry::SharedPtr msg) {
        auto cmd = geometry_msgs::msg::Twist();


        float x = msg->pose.pose.position.x;
        //float y = msg->pose.pose.position.y;
        float z = msg->pose.pose.position.z;

        // yaw회전시 orientation.z와 orientation.w 만 변한다.
        float oz = msg->pose.pose.orientation.z;
        float ow = msg->pose.pose.orientation.w;

        RCLCPP_INFO(this->get_logger(), "x [%.2f] y [%.2f], z [%.2f], oz[%.2f], ow [%.2f]",
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        // 직진
        if (x < 0 && flag == 0) {
            cmd.linear.x = 0.3;
            cmd.angular.z = 0;

        } else
        if (x >= 0 ) {  // x 좌표가 >= 0 이면 멈춘 후 회전한다.
            cmd.linear.x = 0;
            cmd.angular.z = 0.2;
            if ( oz*ow > 0.49 &&  oz*ow < 0.5) //약 90도 회전시 oz와 ow가 0.7 근처가 되므로 곱이 0.49와 0.5 사이면 회전을 멈춤.
            {
              cmd.linear.x = 0;
              cmd.angular.z = 0;
              flag = 1;                        // 90도 회전을 표시하는 플래그
            }
        }
        if (flag == 1) {                       // Crater로 직진
            cmd.linear.x = 0.3;
            cmd.angular.z = 0;

            if (x > 0.5) {                     // x 좌표를 보고 보정한다.
                cmd.linear.x = 0.3;
                cmd.angular.z = 0.1;
            } else
            if (x < 0) {
                cmd.linear.x = 0.3;
                cmd.angular.z = -0.1;
            }

        }

        if (z > 2.38) {                         // z좌표를 보고 Crater 도착을 확인한다.
          cmd.linear.x = 0;
          cmd.angular.z = 0;
          RCLCPP_INFO(this->get_logger(), "Stop! Arrived to Point! ");
        }

        return cmd;
      }


      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
      int flag;


 };

 int main(int argc, char *argv[]) {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<ClimbCrater>();
   RCLCPP_INFO(node->get_logger(), "Rover! Go to crater!");
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
 }
