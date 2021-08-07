#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

class FollowWall : public rclcpp::Node {

    public:
      FollowWall() : Node("FollowWall") {
          sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/rover_5k/laser_scan", 10,
            std::bind(&FollowWall::FollowWall_callback, this, _1));

          pub = this->create_publisher<geometry_msgs::msg::Twist>("/rover_5k/cmd_vel", 10);
      }

    private:
      void FollowWall_callback(sensor_msgs::msg::LaserScan::SharedPtr detection) {
        float toWall = 10;               //센서가 측정할 수있는 최대 거리
        for (int i = 0; i < 200; i++) {  //센서 ray 수
          float distance = detection->ranges[i];
          if (distance < toWall) toWall = distance;  // 가장 작은 거리로 update한다.

        }
        auto command = this->newControlMsg(toWall);
        pub->publish(command);
      }

      geometry_msgs::msg::Twist newControlMsg(float wall) {
        auto cmd = geometry_msgs::msg::Twist();

        RCLCPP_INFO(this->get_logger(), "Distance to wall is : '%f'", wall);

        if (wall < 3.0) {                       // 벽까지 거리가 3m 안으로 들어오면 회전한다.
          cmd.linear.x = 0;
          cmd.angular.z = -0.15;
        } else {                                // 벽까지 거리가 3m 가 넘으면 직진한다. 벽을 따라 가기 위해 왼쪽으로 조금 틀어 준다.
          cmd.linear.x = 0.5;
          cmd.angular.z = 0.03;
        }

        return cmd;
      }

      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowWall>();
  RCLCPP_INFO(node->get_logger(), "If detect the wall, follow it !");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
