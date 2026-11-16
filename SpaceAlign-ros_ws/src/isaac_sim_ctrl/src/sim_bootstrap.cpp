#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "simulation_interfaces/srv/load_world.hpp"
#include "simulation_interfaces/srv/set_simulation_state.hpp"
#include "simulation_interfaces/msg/simulation_state.hpp"

#include <chrono>
#include <cstdlib>

bool call_load_world_srv(std::shared_ptr<rclcpp::Node> node, std::string usdfile_uri)
{
	rclcpp::Client<simulation_interfaces::srv::LoadWorld>::SharedPtr client = node->create_client<simulation_interfaces::srv::LoadWorld>("/load_world");

	auto request = std::make_shared<simulation_interfaces::srv::LoadWorld::Request>();
	request->world_resource.uri = usdfile_uri;

	while (!client->wait_for_service(std::chrono::seconds(1)))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return false;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service '/load_world' not available, waiting again...");
	}

	auto result = client->async_send_request(request);
	// Wait for the result.
	if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service '/load_world'");
		return false;
	}
	else
	{
		auto res_msg = result.get();
		int result_code = res_msg->result.result;
		if (result_code != 1)
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service '/load_world' returned with error code : %d (%s)", result_code, res_msg->result.error_message.c_str());
			return false;
		}
	}
	return true;
}

bool call_set_simulation_state_srv(std::shared_ptr<rclcpp::Node> node, uint state)
{
	rclcpp::Client<simulation_interfaces::srv::SetSimulationState>::SharedPtr client = node->create_client<simulation_interfaces::srv::SetSimulationState>("/set_simulation_state");

	auto request = std::make_shared<simulation_interfaces::srv::SetSimulationState::Request>();
	request->state.state = state;

	while (!client->wait_for_service(std::chrono::seconds(1)))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return false;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service '/set_simulation_state' not available, waiting again...");
	}

	auto result = client->async_send_request(request);
	// Wait for the result.
	if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service '/set_simulation_state'");
		return false;
	}
	return true;
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sim_bootstrap");

	node->declare_parameter("usdfile_uri", "[FILE NOT SPECIFIED]");
	std::string filename = node->get_parameter("usdfile_uri").as_string();
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using: '%s'", filename.c_str());
	
	if(!call_set_simulation_state_srv(node, simulation_interfaces::msg::SimulationState::STATE_STOPPED))
	{
		return -1;
	}

	if(!call_load_world_srv(node, filename))
	{
		return -1;
	}

	if(!call_set_simulation_state_srv(node, simulation_interfaces::msg::SimulationState::STATE_PLAYING))
	{
		return -1;
	}

	rclcpp::shutdown();

	return 0;
}
