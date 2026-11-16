#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cstdlib>

#include "custom_srvs_msgs/action/move_joint.hpp"
#include "custom_srvs_msgs/action/move_pose.hpp"

class TestActionClient : public rclcpp::Node
{
public:
    using MoveJAction = custom_srvs_msgs::action::MoveJoint;
    using MoveLAction = custom_srvs_msgs::action::MovePose;
    using GoalHandleMoveJAction = rclcpp_action::ClientGoalHandle<MoveJAction>;
    using GoalHandleMoveLAction = rclcpp_action::ClientGoalHandle<MoveLAction>;

    explicit TestActionClient(bool use_movej, int cancel_sec)
    : Node("test_action_client")
    {
        this->_move_j_client = rclcpp_action::create_client<MoveJAction>(this, "action/move_j");
        this->_move_l_client = rclcpp_action::create_client<MoveLAction>(this, "action/move_l");

        if (use_movej)
        {
            if (cancel_sec > 0)
                RCLCPP_INFO(this->get_logger(), "Calling 'action/move_j' with cancel after %d sec", cancel_sec);
            else
                RCLCPP_INFO(this->get_logger(), "Calling 'action/move_j' with NO cancel");
            send_movej_goal(cancel_sec);
        }
        else
        {
            if (cancel_sec > 0)
                RCLCPP_INFO(this->get_logger(), "Calling 'action/move_l' with cancel after %d sec", cancel_sec);
            else
                RCLCPP_INFO(this->get_logger(), "Calling 'action/move_l' with NO cancel");
            send_movel_goal(cancel_sec);
        }
    }

    void send_movej_goal(int cancel_sec)
    {
        using namespace std::placeholders;

        if (!this->_move_j_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MoveJAction::Goal();
        std::vector<double> target_joint_values = {d2r(50.0), d2r(-20.0), d2r(110.0), 0.0, d2r(90.0), 0.0};
        goal_msg.joint_target = target_joint_values;
        goal_msg.vel = 0.2;
        goal_msg.acc = 0.2;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveJAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleMoveJAction::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        send_goal_options.feedback_callback = [this](GoalHandleMoveJAction::SharedPtr, const std::shared_ptr<const MoveJAction::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->execution_state.c_str());
        };

        send_goal_options.result_callback = [this](const GoalHandleMoveJAction::WrappedResult & result)
        {
            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
					break;
            }
            std::stringstream ss;
			ss << "[" << (result.result->ok?"SUCCEEDED":"FAILED") << "] Result received: " << result.result->status;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            rclcpp::shutdown();
        };
        this->_move_j_client->async_send_goal(goal_msg, send_goal_options);
		
        if(cancel_sec > 0)
        {
            rclcpp::Rate loop_rate(1.0/((double)cancel_sec));
            loop_rate.sleep();
            RCLCPP_INFO(this->get_logger(), "Sending cancel");
            this->_move_j_client->async_cancel_all_goals();
        }
    }

    void send_movel_goal(int cancel_sec)
    {
        using namespace std::placeholders;

        if (!this->_move_l_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MoveLAction::Goal();
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 1.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;
        target_pose.position.x = -1.000;
        target_pose.position.y = -0.800;
        target_pose.position.z =  0.260;
        goal_msg.target_poses.push_back(target_pose);
        goal_msg.vel = 0.2;
        goal_msg.acc = 0.2;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveLAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](const GoalHandleMoveLAction::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        send_goal_options.feedback_callback = [this](GoalHandleMoveLAction::SharedPtr, const std::shared_ptr<const MoveLAction::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->execution_state.c_str());
        };

        send_goal_options.result_callback = [this](const GoalHandleMoveLAction::WrappedResult & result)
        {
            switch (result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
					break;
            }
            std::stringstream ss;
			ss << "[" << (result.result->ok?"SUCCEEDED":"FAILED") << "] Result received: " << result.result->status;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            rclcpp::shutdown();
        };
        this->_move_l_client->async_send_goal(goal_msg, send_goal_options);
		
        if(cancel_sec > 0)
        {
            rclcpp::Rate loop_rate(1.0/((double)cancel_sec));
            loop_rate.sleep();
            RCLCPP_INFO(this->get_logger(), "Sending cancel");
            this->_move_l_client->async_cancel_all_goals();
        }
    }

private:
    rclcpp_action::Client<MoveJAction>::SharedPtr _move_j_client;
    rclcpp_action::Client<MoveLAction>::SharedPtr _move_l_client;

    double d2r(double value)
    {
        return 3.1415926535897932384626433832795 * value / 180.0;
    }
};  // class TestActionClient

int main(int argc, char * argv[])
{
    bool use_movej = true;
    int cancel_sec = 0;
    if (argc > 1)
    {
        use_movej = strcmp(argv[1], "MOVEJ") == 0;
    }
    if (argc > 2)
    {
        cancel_sec = atoi(argv[2]);
    }

	rclcpp::init(argc, argv);

    auto node = std::make_shared<TestActionClient>(use_movej, cancel_sec);

    rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}
