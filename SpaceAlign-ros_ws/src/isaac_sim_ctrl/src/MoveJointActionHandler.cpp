#include "isaac_sim_ctrl/MoveJointActionHandler.hpp"

MoveJointActionHandler::MoveJointActionHandler(rclcpp::Node *node)
	: _node(node)
{
	_traj_exec_client = rclcpp_action::create_client<ExecuteTrajectory>(_node, "/execute_trajectory");
	_traj_exe_event_pub = _node->create_publisher<std_msgs::msg::String>("/trajectory_execution_event", 10);
}

void MoveJointActionHandler::initialize()
{
	_move_group_vispa = std::make_unique<moveit::planning_interface::MoveGroupInterface>(_node->shared_from_this(), "vispa");

	auto handle_goal = [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJAction::Goal>)
	{
		RCLCPP_INFO(_node->get_logger(), "[MoveJointActionHandler] Received goal request");
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	};

	auto handle_cancel = [this](const std::shared_ptr<GoalHandleMoveJAction>)
	{
		RCLCPP_INFO(_node->get_logger(), "[MoveJointActionHandler] Received request to cancel goal");
		this->cancel_traj_exe();
		return rclcpp_action::CancelResponse::ACCEPT;
	};

	auto handle_accepted = [this](const std::shared_ptr<GoalHandleMoveJAction> goal_handle)
	{
		// this needs to return quickly to avoid blocking the executor,
		// so we declare a lambda function to be called inside a new thread
		auto execute_in_thread = [this, goal_handle]()
		{ return this->execute_MoveJAction(goal_handle); };
		std::thread{execute_in_thread}.detach();
	};

	_movej_server = rclcpp_action::create_server<MoveJAction>(_node, "action/move_j", handle_goal, handle_cancel, handle_accepted);
}

void MoveJointActionHandler::execute_MoveJAction(const std::shared_ptr<GoalHandleMoveJAction> movej_goal_handle)
{
	RCLCPP_INFO(_node->get_logger(), "[MoveJointActionHandler] Executing goal");
	const auto request = movej_goal_handle->get_goal();

	_move_group_vispa->setMaxVelocityScalingFactor(request->vel);
	_move_group_vispa->setMaxAccelerationScalingFactor(request->acc);

	std::vector<std::string> joint_names = {"joint_J1", "joint_J2", "joint_J3", "joint_J4", "joint_J5", "joint_J6"};
	_move_group_vispa->setJointValueTarget(joint_names, request->joint_target);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	auto movej_result = std::make_shared<MoveJAction::Result>();

	if (_move_group_vispa->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
	{
		movej_result->ok = false;
		movej_result->status = "Planning failed";
		movej_goal_handle->abort(movej_result);
		return;
	}

	if (!_traj_exec_client->wait_for_action_server(std::chrono::seconds(5)))
	{
		movej_result->ok = false;
		movej_result->status = "ExecuteTrajectory action server not available";
		movej_goal_handle->abort(movej_result);
		return;
	}

	ExecuteTrajectory::Goal goal;
	goal.trajectory = plan.trajectory;

	rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions options;

	options.goal_response_callback = [this, movej_result, movej_goal_handle](const GoalHandleExecuteTrajectory::SharedPtr &traj_exe_goal_handle)
	{
		if (!traj_exe_goal_handle)
		{
			movej_result->ok = false;
			movej_result->status = "ExecuteTrajectory goal was rejected";
			movej_goal_handle->abort(movej_result);
		}
		else
		{
			RCLCPP_INFO(this->_node->get_logger(), "[MoveJointActionHandler] ExecuteTrajectory goal accepted");
		}
	};

	options.feedback_callback = [movej_goal_handle](GoalHandleExecuteTrajectory::SharedPtr, const std::shared_ptr<const ExecuteTrajectory::Feedback> traj_exe_feedback)
	{
		auto movej_feedback = std::make_shared<MoveJAction::Feedback>();
		movej_feedback->execution_state = traj_exe_feedback->state;
		movej_goal_handle->publish_feedback(movej_feedback);
	};

	// options.result_callback = [this,movej_result,movej_goal_handle](const GoalHandleExecuteTrajectory::WrappedResult & traj_exe_result)
	options.result_callback = [this, movej_result, movej_goal_handle, request](const GoalHandleExecuteTrajectory::WrappedResult &traj_exe_result)
	{
		switch (traj_exe_result.code)
		{
		case rclcpp_action::ResultCode::SUCCEEDED:
			movej_result->ok = true;
			movej_result->status = "Execution finished successfully";
			break;
		case rclcpp_action::ResultCode::ABORTED:
			movej_result->ok = false;
			movej_result->status = "Execution aborted";
			break;
		case rclcpp_action::ResultCode::CANCELED:
			movej_result->ok = false;
			movej_result->status = "Execution canceled";
			break;
		default:
			movej_result->ok = false;
			movej_result->status = "Unknown result code";
			break;
		}
		if (rclcpp::ok())
		{
			// canceled() is only a valid transition while the goal is in the CANCELING state
			if (movej_goal_handle->is_canceling())
				movej_goal_handle->canceled(movej_result);
			else if (movej_result->ok)
				movej_goal_handle->succeed(movej_result);
			else
				movej_goal_handle->abort(movej_result);
		}
		RCLCPP_INFO(this->_node->get_logger(), "[MoveJointActionHandler] MoveIt error code: %d", traj_exe_result.result->error_code.val);
	};

	_traj_exec_client->async_send_goal(goal, options);
}

// For some reason it is NOT possible to cancel the ExecuteTrajectory action.
// As a workaround sending 'stop' on the '/trajectory_execution_event' topic will do the job.
// https://github.com/moveit/moveit2/issues/2808
void MoveJointActionHandler::cancel_traj_exe()
{
	auto message = std_msgs::msg::String();
	message.data = "stop";
	_traj_exe_event_pub->publish(message);
}
