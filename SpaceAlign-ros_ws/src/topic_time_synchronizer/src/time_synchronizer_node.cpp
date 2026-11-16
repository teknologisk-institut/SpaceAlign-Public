#include <functional>
#include <memory>
#include <chrono>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <custom_srvs_msgs/msg/space_robot_state.hpp>

class TimeSynchronizerNode : public rclcpp::Node
{
public:
  TimeSynchronizerNode()
  : Node("time_synchronizer_node")
  {
    auto qos = rclcpp::QoS(10);

    space_state_pub = this->create_publisher<custom_srvs_msgs::msg::SpaceRobotState>("vispa/space_robot_state",qos);

    joint_state_sub.subscribe(this, "/robot_joint_states", qos.get_rmw_qos_profile());
    tool_ft_sub.subscribe(this, "/vispa/tool_ft", qos.get_rmw_qos_profile());
    tool_tcp_sub.subscribe(this,"/vispa/tool_tcp", qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::TimeSynchronizer<
      sensor_msgs::msg::JointState,
      geometry_msgs::msg::WrenchStamped,geometry_msgs::msg::PoseStamped>>(joint_state_sub, tool_ft_sub, tool_tcp_sub, 10);

    sync_->registerCallback(
      std::bind(
        &TimeSynchronizerNode::sync_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(get_logger(), "Time synchronizer node started.");
    RCLCPP_INFO(get_logger(), "Waiting for synced messages on topics: /robot_joint_states and /vispa/tool_ft");
    RCLCPP_INFO(get_logger(), "Publishing state on vispa/space_robot_state");   
      
  }

private:
  void sync_callback(
    const sensor_msgs::msg::JointState::ConstSharedPtr & joint_state_msg,
    const geometry_msgs::msg::WrenchStamped::ConstSharedPtr & tool_ft_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & tool_tcp_msg)
  {
    custom_srvs_msgs::msg::SpaceRobotState space_msg;
    space_msg.header.stamp = this->get_clock()->now();

    space_msg.joint_state = *joint_state_msg;

    /**Published wrench contains the forces and torques imposed by the simulation in order to keep the joints in place.
     * Therfore the is the wrench measurement not the wrench acting on the manipulator but the opposite wrench. To change sign multiply by -1.0*/ 
    geometry_msgs::msg::Wrench w;
    w.force.x = tool_ft_msg->wrench.force.x * -1.0;
    w.force.y = tool_ft_msg->wrench.force.y * -1.0;
    w.force.z = tool_ft_msg->wrench.force.z * -1.0;
    w.torque.x = tool_ft_msg->wrench.torque.x * -1.0;
    w.torque.y = tool_ft_msg->wrench.torque.y * -1.0;
    w.torque.z = tool_ft_msg->wrench.torque.z * -1.0;

    space_msg.tcp_force_torque = w;
    
    space_msg.tcp_pose.position = tool_tcp_msg->pose.position;
    space_msg.tcp_pose.orientation = tool_tcp_msg->pose.orientation;

    space_state_pub->publish(space_msg);

  }
  rclcpp::Publisher<custom_srvs_msgs::msg::SpaceRobotState>::SharedPtr space_state_pub;

  message_filters::Subscriber<sensor_msgs::msg::JointState> joint_state_sub;
  message_filters::Subscriber<geometry_msgs::msg::WrenchStamped> tool_ft_sub;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> tool_tcp_sub;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::JointState,geometry_msgs::msg::WrenchStamped, geometry_msgs::msg::PoseStamped>> sync_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeSynchronizerNode>());
  rclcpp::shutdown();
  return 0;
}
