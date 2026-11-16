#ifndef MOVE_JOINT_ACTION_HANDLER_HPP
#define MOVE_JOINT_ACTION_HANDLER_HPP

#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/string.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "custom_srvs_msgs/action/move_joint.hpp"


class MoveJointActionHandler
{
	public:
		using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
		using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

        using MoveJAction = custom_srvs_msgs::action::MoveJoint;
        using GoalHandleMoveJAction = rclcpp_action::ServerGoalHandle<MoveJAction>;
		
		MoveJointActionHandler(rclcpp::Node* node);
		void initialize();

	private:
		rclcpp::Node* _node;
		rclcpp_action::Server<MoveJAction>::SharedPtr _movej_server;
		rclcpp_action::Client<ExecuteTrajectory>::SharedPtr _traj_exec_client;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _traj_exe_event_pub;
		std::unique_ptr<moveit::planning_interface::MoveGroupInterface> _move_group_vispa;

		void execute_MoveJAction(const std::shared_ptr<GoalHandleMoveJAction> goal_handle);
		void cancel_traj_exe();

		const double joint_tolerance = 0.005;
};

#endif // MOVE_JOINT_ACTION_HANDLER_HPP
