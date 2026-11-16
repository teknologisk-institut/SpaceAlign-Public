#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cstdlib>
#include <std_srvs/srv/empty.hpp>

#include "custom_srvs_msgs/action/move_joint.hpp"
#include "custom_srvs_msgs/action/move_pose.hpp"
#include "simulation_interfaces/srv/reset_simulation.hpp"

using MoveJAction = custom_srvs_msgs::action::MoveJoint;
using MoveLAction = custom_srvs_msgs::action::MovePose;
using GoalHandleMoveJAction = rclcpp_action::ClientGoalHandle<MoveJAction>;
using GoalHandleMoveLAction = rclcpp_action::ClientGoalHandle<MoveLAction>;
using EmptySrv = std_srvs::srv::Empty;
using ResetSimulationSrv = simulation_interfaces::srv::ResetSimulation;

class AllInOneTestClass
{
public:

    explicit AllInOneTestClass() {}

    void init(std::shared_ptr<rclcpp::Node> node)
    {
        this->_move_j_client = rclcpp_action::create_client<MoveJAction>(node, "action/move_j");
        this->_move_l_client = rclcpp_action::create_client<MoveLAction>(node, "action/move_l");

        this->_open_gripper_client = node->create_client<EmptySrv>("/open_gripper");
        this->_close_gripper_client = node->create_client<EmptySrv>("/close_gripper");

        this->_reset_simulation_client = node->create_client<ResetSimulationSrv>("/reset_simulation");
    }

    bool send_movej_goal(std::shared_ptr<rclcpp::Node> node)
    {
        RCLCPP_INFO(node->get_logger(), "send_movej_goal");
        using namespace std::placeholders;

        if (!this->_move_j_client->wait_for_action_server())
        {
            RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MoveJAction::Goal();
        std::vector<double> target_joint_values = {d2r(50.0), d2r(-20.0), d2r(110.0), 0.0, d2r(90.0), 0.0};
        goal_msg.joint_target = target_joint_values;
        goal_msg.vel = 0.2;
        goal_msg.acc = 0.2;

        auto goal_handle_future = _move_j_client->async_send_goal(goal_msg);

        auto status = rclcpp::spin_until_future_complete(node, goal_handle_future);

        if (status != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
            return false;
        }

        auto goal_handle = goal_handle_future.get();

        if (!goal_handle)
        {
            RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
            return false;
        }

        auto result_future = _move_j_client->async_get_result(goal_handle);

        status = rclcpp::spin_until_future_complete(node, result_future);

        if (status != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get result");
            return false;
        }

        auto wrapped_result = result_future.get();

        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node->get_logger(), "Action succeeded");
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Action did not succeed");
        }
        return true;
    }

    bool send_movel_goal(std::shared_ptr<rclcpp::Node> node, const MoveLAction::Goal &goal_msg)
    {
        RCLCPP_INFO(node->get_logger(), "send_movel_goal");
        using namespace std::placeholders;

        if (!this->_move_l_client->wait_for_action_server())
        {
            RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_handle_future = _move_l_client->async_send_goal(goal_msg);

        auto status = rclcpp::spin_until_future_complete(node, goal_handle_future);

        if (status != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
            return false;
        }

        auto goal_handle = goal_handle_future.get();

        if (!goal_handle)
        {
            RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
            return false;
        }

        auto result_future = _move_l_client->async_get_result(goal_handle);

        status = rclcpp::spin_until_future_complete(node, result_future);

        if (status != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get result");
            return false;
        }

        auto wrapped_result = result_future.get();

        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(node->get_logger(), "Action succeeded");
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Action did not succeed");
        }
        return true;
    }

    bool open_gripper(std::shared_ptr<rclcpp::Node> node)
    {
        RCLCPP_INFO(node->get_logger(), "open_gripper");
        return call_EmptySrv(node, _open_gripper_client);
    }

    bool close_gripper(std::shared_ptr<rclcpp::Node> node)
    {
        RCLCPP_INFO(node->get_logger(), "close_gripper");
        return call_EmptySrv(node, _close_gripper_client);
    }

    bool call_EmptySrv(std::shared_ptr<rclcpp::Node> node, rclcpp::Client<EmptySrv>::SharedPtr client)
    {
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<EmptySrv::Request>();
        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to call service open/close gripper");
            return false;
        }
        return true;
    }

    bool reset_sim(std::shared_ptr<rclcpp::Node> node)
    {
        while (!_reset_simulation_client->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<ResetSimulationSrv::Request>();
        auto result = _reset_simulation_client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to call service 'reset_simulation'");
            return false;
        }
        return true;
    }

private:
    rclcpp_action::Client<MoveJAction>::SharedPtr _move_j_client;
    rclcpp_action::Client<MoveLAction>::SharedPtr _move_l_client;
    rclcpp::Client<EmptySrv>::SharedPtr _open_gripper_client;
    rclcpp::Client<EmptySrv>::SharedPtr _close_gripper_client;
    rclcpp::Client<ResetSimulationSrv>::SharedPtr _reset_simulation_client;

    double d2r(double value)
    {
        return 3.1415926535897932384626433832795 * value / 180.0;
    }
};  // class TestActionClient


class MoveLGoalMsg : public MoveLAction::Goal
{
public:
    MoveLGoalMsg(double velocity, double acceleration)
    {
        vel = velocity;
        acc = acceleration;
    }

    void add_pose(double x, double y, double z, double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = qx;
        target_pose.orientation.y = qy;
        target_pose.orientation.z = qz;
        target_pose.orientation.w = qw;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_poses.push_back(target_pose);
    }
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("all_in_one_test");

    AllInOneTestClass aiotc = AllInOneTestClass();
    aiotc.init(node);

    aiotc.reset_sim(node);

    aiotc.send_movej_goal(node);
    aiotc.open_gripper(node);

    rclcpp::WallRate rate(1); // Hz
    rate.sleep();

    MoveLGoalMsg goal_msg = MoveLGoalMsg(0.2, 0.2);
    goal_msg.add_pose(-1.000, -0.800, 0.260, 0.0, 1.0, 0.0, 0.0);
    goal_msg.add_pose(-1.000, -0.800, 0.060, 0.0, 1.0, 0.0, 0.0);
    aiotc.send_movel_goal(node, goal_msg);
    rate.reset();
    rate.sleep();

    aiotc.close_gripper(node);
    rate.reset();
    rate.sleep();

    MoveLGoalMsg goal_msg2 = MoveLGoalMsg(0.2, 0.2);
    goal_msg2.add_pose(-1.000, -0.800, 0.070, 0.0, 1.0, 0.0, 0.0);
    goal_msg2.add_pose( 1.000, -0.800, 0.070, 0.0, 1.0, 0.0, 0.0);
    aiotc.send_movel_goal(node, goal_msg2);
    rate.reset();
    rate.sleep();

    // aiotc.open_gripper(node);
    // rate.reset();
    // rate.sleep();

    // MoveLGoalMsg goal_msg3 = MoveLGoalMsg(0.2, 0.2);
    // goal_msg3.add_pose( 1.000, -0.800, 0.260, 0.0, 1.0, 0.0, 0.0);
    // goal_msg3.add_pose( 0.000, -0.800, 0.260, 0.0, 1.0, 0.0, 0.0);
    // aiotc.send_movel_goal(node, goal_msg3);

    rclcpp::shutdown();

	return 0;
}
