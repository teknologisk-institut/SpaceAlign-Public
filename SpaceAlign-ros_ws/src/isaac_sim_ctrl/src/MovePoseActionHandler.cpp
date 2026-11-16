#include "isaac_sim_ctrl/MovePoseActionHandler.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>


const std::string MovePoseActionHandler::PLANNING_GROUP = "vispa";

MovePoseActionHandler::MovePoseActionHandler(rclcpp::Node* node)
: _node(node)
{
	_traj_exec_client = rclcpp_action::create_client<ExecuteTrajectory>(_node, "/execute_trajectory");
	_traj_exe_event_pub = _node->create_publisher<std_msgs::msg::String>("/trajectory_execution_event", 10);
}

void MovePoseActionHandler::initialize()
{
	_move_group_vispa = std::make_unique<moveit::planning_interface::MoveGroupInterface>(_node->shared_from_this(), PLANNING_GROUP);
	// --- DEBUG START -------------------------------------------------------------------------------------------------------------------------
	RCLCPP_INFO(_node->get_logger(), "-------------------------------------------------------------------------------------------------------------------------");
	const srdf::ModelConstSharedPtr srdf_model = _move_group_vispa->getRobotModel()->getSRDF();
	RCLCPP_INFO(_node->get_logger(), "SRDF model name: %s", srdf_model->getName().c_str());

	RCLCPP_INFO(_node->get_logger(), "Groups:");
	std::vector<srdf::Model::Group>::const_iterator it;
	for(it=srdf_model->getGroups().begin(); it!=srdf_model->getGroups().end(); it++)
	{
		RCLCPP_INFO(_node->get_logger(), "\t%s", it->name_.c_str());
	}

	RCLCPP_INFO(_node->get_logger(), "GroupStates:");
	std::vector<srdf::Model::GroupState>::const_iterator it2;
	for(it2=srdf_model->getGroupStates().begin(); it2!=srdf_model->getGroupStates().end(); it2++)
	{
		RCLCPP_INFO(_node->get_logger(), "\t%s\tgroup: %s", it2->name_.c_str(), it2->group_.c_str());
	}

	RCLCPP_INFO(_node->get_logger(), "JointModels:");
	std::vector<const moveit::core::JointModel*>::const_iterator joint_model_it;
	for(joint_model_it=_move_group_vispa->getRobotModel()->getJointModels().begin(); joint_model_it!=_move_group_vispa->getRobotModel()->getJointModels().end(); joint_model_it++)
	{
		moveit::core::JointModel::Bounds bounds = (*joint_model_it)->getVariableBounds();
		moveit::core::JointModel::Bounds::iterator bounds_it;
		std::string bound_str = "";
		for(bounds_it=bounds.begin(); bounds_it!=bounds.end(); bounds_it++)
		{
			bound_str += bounds_it->position_bounded_ ? "POS " : "";
			bound_str += bounds_it->velocity_bounded_ ? "VEL " : "";
			bound_str += bounds_it->acceleration_bounded_ ? "ACC " : "";
			bound_str += bounds_it->jerk_bounded_ ? "JERK " : "";
		}
		RCLCPP_INFO(_node->get_logger(), "\t%s\t#bounds: %d (%s)", (*joint_model_it)->getName().c_str(), (int)bounds.size(), bound_str.c_str());
	}
	RCLCPP_INFO(_node->get_logger(), "-------------------------------------------------------------------------------------------------------------------------");
	// --- DEBUG END ---------------------------------------------------------------------------------------------------------------------------

	auto handle_goal = [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveLAction::Goal>)
    {
		RCLCPP_INFO(_node->get_logger(), "[MovePoseActionHandler] Received goal request");
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [this](const std::shared_ptr<GoalHandleMoveLAction>)
    {
		RCLCPP_INFO(_node->get_logger(), "[MovePoseActionHandler] Received request to cancel goal");
		this->cancel_traj_exe();
		return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](const std::shared_ptr<GoalHandleMoveLAction> goal_handle)
    {
		// this needs to return quickly to avoid blocking the executor,
		// so we declare a lambda function to be called inside a new thread
		auto execute_in_thread = [this, goal_handle](){return this->execute_MoveLAction(goal_handle);};
		std::thread{execute_in_thread}.detach();
    };

	_movel_server = rclcpp_action::create_server<MoveLAction>(_node, "action/move_l", handle_goal, handle_cancel, handle_accepted);
}

void MovePoseActionHandler::execute_MoveLAction(const std::shared_ptr<GoalHandleMoveLAction> movel_goal_handle)
{
	RCLCPP_INFO(_node->get_logger(), "[MovePoseActionHandler] Executing goal");

	auto current_state = _move_group_vispa->getCurrentState();
    if (!current_state)
    {
		RCLCPP_ERROR(_node->get_logger(), "Failed to get current robot state");
		return;
    }

	const auto request = movel_goal_handle->get_goal();
	auto movel_result = std::make_shared<MoveLAction::Result>();

	_move_group_vispa->setMaxVelocityScalingFactor(request->vel);
	_move_group_vispa->setMaxAccelerationScalingFactor(request->acc);
	_move_group_vispa->setStartStateToCurrentState();

	// Compute Cartesian path
	moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
	const double eef_step = 0.005;      // 5 mm interpolation
	double fraction = _move_group_vispa->computeCartesianPath(request->target_poses, eef_step, cartesian_trajectory);
	if (fraction <= 0.99)
	{
		RCLCPP_ERROR(_node->get_logger(), "Cartesian path planning failed. Fraction achieved: %.3f", fraction);
		movel_result->ok = false;
		movel_result->status = "Cartesian path planning failed. Fraction below 0.99";
		movel_goal_handle->abort(movel_result);
		return;
	}

	// print_trajectory(cartesian_trajectory);

	// Time-parameterize the Cartesian trajectory before execution
    robot_trajectory::RobotTrajectory rt(_move_group_vispa->getRobotModel(), PLANNING_GROUP);

    rt.setRobotTrajectoryMsg(*current_state, cartesian_trajectory);

    trajectory_processing::TimeOptimalTrajectoryGeneration  time_param;
    bool timing_ok = time_param.computeTimeStamps(rt, request->vel, request->acc);

    if (!timing_ok)
    {
		RCLCPP_ERROR(_node->get_logger(), "Failed to time-parameterize trajectory");
		return;
    }

    rt.getRobotTrajectoryMsg(cartesian_trajectory);
	
	// print_trajectory(cartesian_trajectory);
	
	if (!_traj_exec_client->wait_for_action_server(std::chrono::seconds(5)))
	{
		movel_result->ok = false;
		movel_result->status = "ExecuteTrajectory action server not available";
		movel_goal_handle->abort(movel_result);
		return;
	}

	ExecuteTrajectory::Goal goal;
	goal.trajectory = cartesian_trajectory;

	rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions options;

	options.goal_response_callback = [this, movel_result, movel_goal_handle](const GoalHandleExecuteTrajectory::SharedPtr & traj_exe_goal_handle)
	{
		if (!traj_exe_goal_handle)
		{
			movel_result->ok = false;
			movel_result->status = "ExecuteTrajectory goal was rejected";
			movel_goal_handle->abort(movel_result);
		}
		else
		{
			RCLCPP_INFO(this->_node->get_logger(), "[MovePoseActionHandler] ExecuteTrajectory goal accepted");
		}
	};

	options.feedback_callback = [movel_goal_handle](GoalHandleExecuteTrajectory::SharedPtr, const std::shared_ptr<const ExecuteTrajectory::Feedback> traj_exe_feedback)
	{
		auto movel_feedback = std::make_shared<MoveLAction::Feedback>();
		movel_feedback->execution_state = traj_exe_feedback->state;
		movel_goal_handle->publish_feedback(movel_feedback);
	};

	options.result_callback = [this,movel_result,movel_goal_handle](const GoalHandleExecuteTrajectory::WrappedResult & traj_exe_result)
	{
		switch (traj_exe_result.code)
		{
		case rclcpp_action::ResultCode::SUCCEEDED:
			movel_result->ok = true;
			movel_result->status = "Execution finished successfully";
			break;
		case rclcpp_action::ResultCode::ABORTED:
			movel_result->ok = false;
			movel_result->status = "Execution aborted";
			break;
		case rclcpp_action::ResultCode::CANCELED:
			movel_result->ok = false;
			movel_result->status = "Execution canceled";
			break;
		default:
			movel_result->ok = false;
			movel_result->status = "Unknown result code";
			break;
		}
		if (rclcpp::ok())
		{
			// canceled() is only a valid transition while the goal is in the CANCELING state
			if (movel_goal_handle->is_canceling())
				movel_goal_handle->canceled(movel_result);
			else if (movel_result->ok)
				movel_goal_handle->succeed(movel_result);
			else
				movel_goal_handle->abort(movel_result);
		}

		RCLCPP_INFO(this->_node->get_logger(), "[MovePoseActionHandler] MoveIt error code: %d", traj_exe_result.result->error_code.val);
	};

	_traj_exec_client->async_send_goal(goal, options);
}

// For some reason it is NOT possible to cancel the ExecuteTrajectory action.
// As a workaround sending 'stop' on the '/trajectory_execution_event' topic will do the job.
// https://github.com/moveit/moveit2/issues/2808
void MovePoseActionHandler::cancel_traj_exe()
{
	auto message = std_msgs::msg::String();
	message.data = "stop";
	_traj_exe_event_pub->publish(message);
}

void MovePoseActionHandler::print_trajectory(const moveit_msgs::msg::RobotTrajectory &trajectory)
{
	int n = (int)trajectory.joint_trajectory.points.size();
	RCLCPP_INFO(_node->get_logger(), "-------------------------------------------------------------------------------------------------------------------------");
	RCLCPP_INFO(_node->get_logger(), "Num JointTrajectoryPoints: %d", n);
	for(int i=0; i<n; i++)
	{
		std::stringstream sStr;
		bool first_time = true;
		int n2 = (int)trajectory.joint_trajectory.points[i].positions.size();
		for(int j=0; j<n2; j++)
		{
			if(first_time)
			{
				first_time = false;
				sStr << trajectory.joint_trajectory.points[i].positions[j];
			}
			else
			{
				sStr << ", " << trajectory.joint_trajectory.points[i].positions[j];
			}
		}
		RCLCPP_INFO(_node->get_logger(), "\t[%s]", sStr.str().c_str());
	}
	RCLCPP_INFO(_node->get_logger(), "-------------------------------------------------------------------------------------------------------------------------");
}