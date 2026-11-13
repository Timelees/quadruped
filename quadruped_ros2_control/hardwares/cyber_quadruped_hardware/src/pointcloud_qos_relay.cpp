#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cyber_quadruped_hardware
{

class PointCloudQosRelay final : public rclcpp::Node
{
public:
  PointCloudQosRelay()
  : Node("pointcloud_qos_relay")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/cyberdog_1/camera/depth/color/points");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/cyberdog_1/perception/depth/color/points_reliable");
    const auto queue_size = declare_parameter<int>("queue_size", 1);

    const auto depth = static_cast<size_t>(queue_size > 0 ? queue_size : 1);

    rclcpp::SensorDataQoS sub_qos;
    sub_qos.keep_last(depth);
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, pub_qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, sub_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
      {
        publisher_->publish(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Relaying point cloud QoS: %s (best_effort) -> %s (reliable), queue_size=%zu",
      input_topic.c_str(), output_topic.c_str(), depth);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace cyber_quadruped_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cyber_quadruped_hardware::PointCloudQosRelay>());
  rclcpp::shutdown();
  return 0;
}
