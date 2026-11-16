#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdlib>

#include "custom_srvs_msgs/srv/motion_traj.hpp"
#include "custom_srvs_msgs/srv/ptp_joint_value_target.hpp"

using MotionTrajSrv = custom_srvs_msgs::srv::MotionTraj;
using PTPJointValueTargetSrv = custom_srvs_msgs::srv::PTPJointValueTarget;

double d2r(double value)
{
    return 3.1415926535897932384626433832795 * value / 180.0;
}

bool call_move_ptp(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::Client<PTPJointValueTargetSrv>::SharedPtr client = node->create_client<PTPJointValueTargetSrv>("move_ptp_jpos");

	auto request = std::make_shared<PTPJointValueTargetSrv::Request>();
    std::vector<double> target_joint_values = {d2r(50.0), d2r(-20.0), d2r(110.0), 0.0, d2r(90.0), 0.0};
	request->joint_target = target_joint_values;
    request->vel = 0.2;
    request->acc = 0.2;

	while (!client->wait_for_service(std::chrono::seconds(1)))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return false;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	auto result = client->async_send_request(request);
	// Wait for the result.
	if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service 'move_ptp'");
		return false;
	}
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result of 'move_ptp': %d", result.get()->ok);
	return true;
}

bool call_move_lin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::Client<MotionTrajSrv>::SharedPtr client = node->create_client<MotionTrajSrv>("move_lin");

	auto request = std::make_shared<MotionTrajSrv::Request>();
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 1.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;
    target_pose.position.x = -1.000;
    target_pose.position.y = -0.800;
    target_pose.position.z =  0.260;
	request->target_poses.push_back(target_pose);
    request->vel = 0.2;
    request->acc = 0.2;

	while (!client->wait_for_service(std::chrono::seconds(1)))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return false;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	auto result = client->async_send_request(request);
	// Wait for the result.
	if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service 'move_lin'");
		return false;
	}
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result of 'move_lin': %d", result.get()->ok);
	return true;
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_srvs");
	
	if(!call_move_ptp(node))
	{
		return -1;
	}

    rclcpp::WallRate rate(1); // Hz
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-- TICK ---");
    rate.sleep();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-- TICK ---");

    if(!call_move_lin(node))
	{
		return -1;
	}

	rclcpp::shutdown();

	return 0;
}
