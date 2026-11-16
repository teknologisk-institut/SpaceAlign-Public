#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

#include "isaac_sim_ctrl/SimCtrl.hpp"
#include "isaac_sim_ctrl/USDA_writer.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

using namespace std::chrono_literals;

SimCtrl::SimCtrl()
 : Node("sim_ctrl"),
   _gripper_open(false),
   _moveJointActHndl(this),
   _movePoseActHndl(this)
{
	this->declare_parameter("tmp_usdpath", "");
	this->declare_parameter("tmp_usdpath_win", "");

	_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

	rclcpp::SubscriptionOptions subscription_options;
	subscription_options.callback_group = _callback_group;
	_joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
		"/robot_joint_states", 10, std::bind(&SimCtrl::robot_joint_states, this, std::placeholders::_1), subscription_options);
	_robot_joint_states_pub = this->create_publisher<sensor_msgs::msg::JointState>("/robot_joint_commands", 10);
	_joint_commands_reset_pub = this->create_publisher<sensor_msgs::msg::JointState>(
		"/robot_joint_commands/reset", rclcpp::QoS(1).reliable());

    _get_entities_states_client = this->create_client<GetEntitiesStatesSrv>(
		"/get_entities_states",
		rclcpp::SystemDefaultsQoS(),
		_callback_group
	);

    _set_sim_state_client = this->create_client<SetSimulationStateSrv>(
		"/set_simulation_state",
		rclcpp::SystemDefaultsQoS(),
		_callback_group
	);

	_load_world_client = this->create_client<LoadWorldSrv>(
		"/load_world",
		rclcpp::SystemDefaultsQoS(),
		_callback_group
	);

	_switch_controller_client = this->create_client<SwitchControllerSrv>(
		"/controller_manager/switch_controller",
		rclcpp::ServicesQoS(),
		_callback_group
	);
}

void SimCtrl::initialize()
{
    _move_group_vispa = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "vispa");

	_move_ptp_srv = this->create_service<MotionTrajSrv>("move_ptp", std::bind(&SimCtrl::move_ptp, this, std::placeholders::_1, std::placeholders::_2));
	_move_ptp_jpos_srv = this->create_service<PTPJointValueTargetSrv>("move_ptp_jpos", std::bind(&SimCtrl::move_ptp_jpos, this, std::placeholders::_1, std::placeholders::_2));
	_move_lin_srv = this->create_service<MotionTrajSrv>("move_lin", std::bind(&SimCtrl::move_lin, this, std::placeholders::_1, std::placeholders::_2));

	_open_gripper_srv = this->create_service<EmptySrv>("open_gripper", std::bind(&SimCtrl::open_gripper, this, std::placeholders::_1, std::placeholders::_2));
	_close_gripper_srv = this->create_service<EmptySrv>("close_gripper", std::bind(&SimCtrl::close_gripper, this, std::placeholders::_1, std::placeholders::_2));

	_save_simulation_state_srv = this->create_service<SaveSimulationStateSrv>(
		"save_simulation_state",
		std::bind(&SimCtrl::save_simulation_state, this, std::placeholders::_1, std::placeholders::_2),
		rclcpp::SystemDefaultsQoS(),
		_callback_group
	);
	_load_simulation_state_srv = this->create_service<LoadSimulationStateSrv>("load_simulation_state", std::bind(&SimCtrl::load_simulation_state, this, std::placeholders::_1, std::placeholders::_2));

	_moveJointActHndl.initialize();
	_movePoseActHndl.initialize();

	auto robot_joint_states_timer_callback = [this]() -> void
	{
		sensor_msgs::msg::JointState msg = sensor_msgs::msg::JointState();
		msg.header.stamp = this->get_clock()->now();
		msg.name = {"Slider_1", "Slider_2"};
		if(this->_gripper_open)
			msg.position = {0.03, -0.03};
		else
			msg.position = {0.0, 0.0};
		this->_robot_joint_states_pub->publish(msg);
	};
	_robot_joint_states_timer = this->create_wall_timer(50ms, robot_joint_states_timer_callback);
}

void SimCtrl::robot_joint_states(sensor_msgs::msg::JointState::UniquePtr msg)
{
	std::lock_guard<std::mutex> lock(_robot_joint_states_mtx);
	_robot_joint_states_pos = msg->position;
	++_robot_joint_states_update_count;
}

bool SimCtrl::movej(std::vector<double> target_joint_values, double vel_factor, double acc_factor)
{
	_move_group_vispa->setMaxVelocityScalingFactor(vel_factor);
	_move_group_vispa->setMaxAccelerationScalingFactor(acc_factor);

	std::vector<std::string> joint_names = {"joint_J1", "joint_J2", "joint_J3", "joint_J4", "joint_J5", "joint_J6"};
	_move_group_vispa->setJointValueTarget(joint_names, target_joint_values);
	_move_group_vispa->setJointValueTarget(target_joint_values);

	// Plan to target
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = (_move_group_vispa->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

	if (success)
	{
		_move_group_vispa->execute(plan);
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "MoveIt planning failed");
		return false;
	}
	return true;
}

void SimCtrl::move_ptp(const std::shared_ptr<MotionTrajSrv::Request> request, std::shared_ptr<MotionTrajSrv::Response> response)
{
	_move_group_vispa->setMaxVelocityScalingFactor(request->vel);
	_move_group_vispa->setMaxAccelerationScalingFactor(request->acc);

	_move_group_vispa->setPoseTargets(request->target_poses);

	// Plan to target
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = (_move_group_vispa->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

	if (success)
	{
		_move_group_vispa->execute(plan);
		response->ok = true;
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "MoveIt planning failed");
		response->ok = false;
	}
}

void SimCtrl::move_ptp_jpos(const std::shared_ptr<PTPJointValueTargetSrv::Request> request, std::shared_ptr<PTPJointValueTargetSrv::Response> response)
{
	_move_group_vispa->setMaxVelocityScalingFactor(request->vel);
	_move_group_vispa->setMaxAccelerationScalingFactor(request->acc);

	std::vector<std::string> joint_names = {"joint_J1", "joint_J2", "joint_J3", "joint_J4", "joint_J5", "joint_J6"};
	_move_group_vispa->setJointValueTarget(joint_names, request->joint_target);

	// Plan to target
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = (_move_group_vispa->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

	if (success)
	{
		_move_group_vispa->execute(plan);
		response->ok = true;
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "MoveIt planning failed");
		response->ok = false;
	}
}

void SimCtrl::move_lin(const std::shared_ptr<MotionTrajSrv::Request> request, std::shared_ptr<MotionTrajSrv::Response> response)
{
	_move_group_vispa->setMaxVelocityScalingFactor(request->vel);
	_move_group_vispa->setMaxAccelerationScalingFactor(request->acc);

	// Compute Cartesian path
	moveit_msgs::msg::RobotTrajectory trajectory;
	const double eef_step = 0.005;      // 5 mm interpolation

	double fraction = _move_group_vispa->computeCartesianPath(request->target_poses, eef_step, trajectory);

	// --- DEBUG START -------------------------------------------------------------------------------------------------------------------------
	int n = (int)trajectory.joint_trajectory.points.size();
	RCLCPP_INFO(this->get_logger(), "[SimCtrl::move_lin] Num JointTrajectoryPoints: %d", n);
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
		RCLCPP_INFO(this->get_logger(), "\t[%s]", sStr.str().c_str());
	}
	// --- DEBUG END ---------------------------------------------------------------------------------------------------------------------------

	if (fraction > 0.99)
	{
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory = trajectory;
		_move_group_vispa->execute(plan);
		response->ok = true;
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Cartesian path planning failed. Fraction achieved: %.3f", fraction);
		response->ok = false;
	}
}

void SimCtrl::open_gripper(const std::shared_ptr<EmptySrv::Request>, std::shared_ptr<EmptySrv::Response>)
{
	RCLCPP_INFO(this->get_logger(), "[SimCtrl::open_gripper] Open gripper");
	this->_gripper_open = true;
}

void SimCtrl::close_gripper(const std::shared_ptr<EmptySrv::Request>, std::shared_ptr<EmptySrv::Response>)
{
	RCLCPP_INFO(this->get_logger(), "[SimCtrl::open_gripper] Close gripper");
	this->_gripper_open = false;
}

void SimCtrl::geometry_msgs_Pose_to_eigen(const geometry_msgs::msg::Pose &pose, Eigen::Matrix4d &matrix)
{
	Eigen::Quaterniond q;
	q.x() = pose.orientation.x;
	q.y() = pose.orientation.y;
	q.z() = pose.orientation.z;
	q.w() = pose.orientation.w;

	Eigen::Vector3d t;
	t.x() = pose.position.x;
	t.y() = pose.position.y;
	t.z() = pose.position.z;

	matrix = Eigen::Matrix4d::Identity();
	matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
	matrix.block<3, 1>(0, 3) = t;
}

void SimCtrl::save_simulation_state(const std::shared_ptr<SaveSimulationStateSrv::Request>, std::shared_ptr<SaveSimulationStateSrv::Response> save_sim_response)
{
	RCLCPP_INFO(this->get_logger(), "[SimCtrl::save_simulation_state] --->");

	std::vector<double> joint_pos;
	{
		std::lock_guard<std::mutex> lock(_robot_joint_states_mtx);
		joint_pos = _robot_joint_states_pos;
	}

	while (!_get_entities_states_client->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			save_sim_response->ok = false;
			return;
		}
		RCLCPP_INFO(this->get_logger(), "External service not available, waiting again...");
	}

	auto entities_states_request = std::make_shared<GetEntitiesStatesSrv::Request>();
	entities_states_request->filters.filter = "/World/";
	auto future_result = _get_entities_states_client->async_send_request(entities_states_request);

	std::future_status status;
	do
	{
		status = future_result.wait_for(10ms);
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Node shut down during wait. Exiting.");
			save_sim_response->ok = false;
			return;
		}
	}
	while (status != std::future_status::ready);

	std::string usdpath = this->get_parameter("tmp_usdpath").as_string();

	auto entities_states_response = future_result.get();
	save_sim_response->usda_file = USDA_writer::write(usdpath, joint_pos, entities_states_response);

	save_sim_response->ok = true;
	RCLCPP_INFO(this->get_logger(), "[SimCtrl::save_simulation_state] <---");
}

void SimCtrl::load_simulation_state(const std::shared_ptr<LoadSimulationStateSrv::Request> request, std::shared_ptr<LoadSimulationStateSrv::Response> response)
{
	RCLCPP_INFO(this->get_logger(), "[SimCtrl::load_simulation_state] loading: %s", request->usda_file.c_str());

	std::string usdpath = this->get_parameter("tmp_usdpath").as_string();
	size_t n1 = request->usda_file.rfind("__");
	std::string json_file = usdpath + request->usda_file.substr(0, n1) + "__jointstate.json";

	std::ifstream ifs(json_file);
	if (!ifs)
	{
		RCLCPP_ERROR(this->get_logger(), "Could not open saved joint state: '%s'", json_file.c_str());
		response->ok = false;
		return;
	}

	std::vector<double> saved_joint_positions;
	try
	{
		nlohmann::json jdata = nlohmann::json::parse(ifs);
		saved_joint_positions = jdata.at("joint_target").get<std::vector<double>>();
	}
	catch (const std::exception &error)
	{
		RCLCPP_ERROR(this->get_logger(), "Invalid saved joint state '%s': %s", json_file.c_str(), error.what());
		response->ok = false;
		return;
	}

	if (saved_joint_positions.size() < 6)
	{
		RCLCPP_ERROR(this->get_logger(), "Saved joint state contains %zu positions; expected at least 6", saved_joint_positions.size());
		response->ok = false;
		return;
	}

	if (!call_switch_controller(false))
	{
		response->ok = false;
		return;
	}

	if(!call_set_simulation_state_srv(simulation_interfaces::msg::SimulationState::STATE_STOPPED))
	{
		response->ok = false;
		return;
	}

	usdpath = this->get_parameter("tmp_usdpath_win").as_string();
	if(usdpath.empty())
		usdpath = this->get_parameter("tmp_usdpath").as_string();
	std::string usda_file = usdpath + request->usda_file;

	if(!call_load_world_srv(usda_file))
	{
		response->ok = false;
		return;
	}

	sensor_msgs::msg::JointState reset_command;
	reset_command.header.stamp = this->get_clock()->now();
	reset_command.name = {"joint_J1", "joint_J2", "joint_J3", "joint_J4", "joint_J5", "joint_J6"};
	reset_command.position.assign(saved_joint_positions.begin(), saved_joint_positions.begin() + 6);
	_joint_commands_reset_pub->publish(reset_command);

	std::size_t previous_update_count;
	{
		std::lock_guard<std::mutex> lock(_robot_joint_states_mtx);
		previous_update_count = _robot_joint_states_update_count;
	}

	if(!call_set_simulation_state_srv(simulation_interfaces::msg::SimulationState::STATE_PLAYING))
	{
		response->ok = false;
		return;
	}

	if (!wait_for_joint_state(reset_command.position, previous_update_count))
	{
		RCLCPP_ERROR(this->get_logger(), "Timed out waiting for the restored joint state; leaving vispa_controller inactive");
		response->ok = false;
		return;
	}

	if (!call_switch_controller(true))
	{
		response->ok = false;
		return;
	}

	response->ok = true;
}

bool SimCtrl::call_switch_controller(bool activate)
{
	while (!_switch_controller_client->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the controller manager. Exiting.");
			return false;
		}
		RCLCPP_INFO(this->get_logger(), "Controller manager not available, waiting again...");
	}

	auto request = std::make_shared<SwitchControllerSrv::Request>();
	if (activate)
		request->activate_controllers = {"vispa_controller"};
	else
		request->deactivate_controllers = {"vispa_controller"};
	request->strictness = SwitchControllerSrv::Request::BEST_EFFORT;
	request->activate_asap = true;
	request->timeout.sec = 2;

	auto future_result = _switch_controller_client->async_send_request(request);
	while (future_result.wait_for(10ms) != std::future_status::ready)
	{
		if (!rclcpp::ok())
			return false;
	}

	auto switch_response = future_result.get();
	if (!switch_response->ok)
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to %s vispa_controller: %s", activate ? "activate" : "deactivate",
			switch_response->message.c_str());
	}
	return switch_response->ok;
}

bool SimCtrl::wait_for_joint_state(const std::vector<double> &target, std::size_t previous_update_count)
{
	const auto deadline = std::chrono::steady_clock::now() + 4s;
	while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
	{
		{
			std::lock_guard<std::mutex> lock(_robot_joint_states_mtx);
			if (_robot_joint_states_update_count > previous_update_count && _robot_joint_states_pos.size() >= target.size())
			{
				bool restored = true;
				for (std::size_t i = 0; i < target.size(); ++i)
					restored = restored && std::abs(_robot_joint_states_pos[i] - target[i]) < 0.01;
				if (restored)
					return true;
			}
		}
		std::this_thread::sleep_for(10ms);
	}
	return false;
}

bool SimCtrl::call_set_simulation_state_srv(uint state)
{
	while (!_set_sim_state_client->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return false;
		}
		RCLCPP_INFO(this->get_logger(), "External service not available, waiting again...");
	}

	auto request = std::make_shared<SetSimulationStateSrv::Request>();
	request->state.state = state;
	auto future_result = _set_sim_state_client->async_send_request(request);

	std::future_status status;
	do
	{
		status = future_result.wait_for(10ms);
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Node shut down during wait. Exiting.");
			return false;
		}
	}
	while (status != std::future_status::ready);

	auto response = future_result.get();

	return true;
}

bool SimCtrl::call_load_world_srv(std::string usdfile_uri)
{
	RCLCPP_INFO(this->get_logger(), "Loading USDA file: '%s'.", usdfile_uri.c_str());

	while (!_load_world_client->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			return false;
		}
		RCLCPP_INFO(this->get_logger(), "External service not available, waiting again...");
	}

	auto request = std::make_shared<LoadWorldSrv::Request>();
	request->world_resource.uri = usdfile_uri;
	auto future_result = _load_world_client->async_send_request(request);

	std::future_status status;
	do
	{
		status = future_result.wait_for(10ms);
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Node shut down during wait. Exiting.");
			return false;
		}
	}
	while (status != std::future_status::ready);

	auto response = future_result.get();
	int result_code = response->result.result;
	if (result_code != 1)
	{
		RCLCPP_ERROR(this->get_logger(), "Service '/load_world' returned with error code : %d (%s)", result_code, response->result.error_message.c_str());
		return false;
	}

	return true;
}
