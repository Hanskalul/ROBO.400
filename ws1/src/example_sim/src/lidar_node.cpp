#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
  : Node("lidar_node")
  {
    auto topic_callback =
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) -> void {
        auto data = geometry_msgs::msg::Twist();

        bool allMore = true;
        for (int i = 0; i < sizeof(msg->ranges); i++)
        {
          if (msg->ranges[i] < 1.0)
          {
            allMore = false;
            break;
          }
        }
        if (allMore) //if all bigger than one
        {
          data.linear.x = 0.5;
          data.angular.z = 0.0;
          // data.mutable_linear()->set_x(0.5);
          // data.mutable_angular()->set_z(0.0);
        }
        else
        {
          data.linear.x = 0.0;
          data.angular.z = 0.5;
          // data.mutable_linear()->set_x(0.0);
          // data.mutable_angular()->set_z(0.5);
        }
        this->publisher_->publish(data);
      };
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, topic_callback);
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
