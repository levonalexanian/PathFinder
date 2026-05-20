#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace pathfinder_car
{

class GzPoseRelayNode : public rclcpp::Node
{
public:
  GzPoseRelayNode()
  : rclcpp::Node("gz_pose_relay"),
    broadcaster_(*this)
  {
    declare_parameter<std::string>("source_topic", "/model/pathfinder_car/pose");
    declare_parameter<std::string>("parent_frame", "map");
    declare_parameter<std::string>("child_frame", "base_link");

    source_topic_ = get_parameter("source_topic").as_string();
    parent_frame_ = get_parameter("parent_frame").as_string();
    child_frame_ = get_parameter("child_frame").as_string();

    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      source_topic_, rclcpp::QoS(50),
      std::bind(&GzPoseRelayNode::on_msg, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "relaying %s -> /tf as %s -> %s",
      source_topic_.c_str(), parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void on_msg(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped out;
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      out.header.stamp = now();
    } else {
      out.header.stamp = msg->header.stamp;
    }
    out.header.frame_id = parent_frame_;
    out.child_frame_id = child_frame_;
    out.transform.translation.x = msg->pose.position.x;
    out.transform.translation.y = msg->pose.position.y;
    out.transform.translation.z = msg->pose.position.z;
    out.transform.rotation = msg->pose.orientation;
    broadcaster_.sendTransform(out);
  }

  std::string source_topic_;
  std::string parent_frame_;
  std::string child_frame_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  tf2_ros::TransformBroadcaster broadcaster_;
};

}  // namespace pathfinder_car

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pathfinder_car::GzPoseRelayNode>());
  rclcpp::shutdown();
  return 0;
}
