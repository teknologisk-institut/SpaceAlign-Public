#ifndef MOVE_POSE_ACTION_HANDLER_HPP
#define MOVE_POSE_ACTION_HANDLER_HPP

#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/string.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "custom_srvs_msgs/action/move_pose.hpp"


class MovePoseActionHandler
{
	public:
		using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
		using GoalHandleExecuteTrajectory = rclcpp_action::ClientGoalHandle<ExecuteTrajectory>;

        using MoveLAction = custom_srvs_msgs::action::MovePose;
        using GoalHandleMoveLAction = rclcpp_action::ServerGoalHandle<MoveLAction>;
		
		MovePoseActionHandler(rclcpp::Node* node);
		void initialize();

	private:
		static const std::string PLANNING_GROUP;

		rclcpp::Node* _node;
		rclcpp_action::Server<MoveLAction>::SharedPtr _movel_server;
		rclcpp_action::Client<ExecuteTrajectory>::SharedPtr _traj_exec_client;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _traj_exe_event_pub;
		std::unique_ptr<moveit::planning_interface::MoveGroupInterface> _move_group_vispa;

		void execute_MoveLAction(const std::shared_ptr<GoalHandleMoveLAction> goal_handle);
		void cancel_traj_exe();
		void print_trajectory(const moveit_msgs::msg::RobotTrajectory &trajectory);
};

#endif // MOVE_POSE_ACTION_HANDLER_HPP
