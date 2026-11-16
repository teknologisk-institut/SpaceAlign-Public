#ifndef MAINCONTROLLER_HPP
#define MAINCONTROLLER_HPP

#include <memory>
#include <chrono>
#include <cstdlib>
#include <string>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "custom_srvs_msgs/action/move_joint.hpp"
#include "custom_srvs_msgs/action/move_pose.hpp"
#include "custom_srvs_msgs/msg/space_robot_state.hpp"
#include "custom_srvs_msgs/srv/save_simulation_state.hpp"
#include "custom_srvs_msgs/srv/load_simulation_state.hpp"

#include <MAPLE_K-1.0/MAL.hpp>
#include <MAPLE_K-1.0/Measurement.hpp>

using MoveJAction = custom_srvs_msgs::action::MoveJoint;
using MoveLAction = custom_srvs_msgs::action::MovePose;
using GoalHandleMoveJAction = rclcpp_action::ClientGoalHandle<MoveJAction>;
using GoalHandleMoveLAction = rclcpp_action::ClientGoalHandle<MoveLAction>;
using EmptySrv = std_srvs::srv::Empty;

namespace ROS_INTERFACE
{

    enum MoveState
    {
        ERROR_MODE,
        undefined,
        moveL,
        moveJ,
        IDLE,
        STOPPING,
        STOPPED
    };

    class MainController : public rclcpp::Node
    {
    private:
        std::atomic<bool> reached_target{false};
        rclcpp_action::Client<MoveJAction>::SharedPtr move_j_client;
        rclcpp_action::Client<MoveLAction>::SharedPtr move_l_client;
        rclcpp::Client<EmptySrv>::SharedPtr open_gripper_srv;
        rclcpp::Client<EmptySrv>::SharedPtr close_gripper_srv;

        rclcpp_action::Client<MoveLAction>::SendGoalOptions moveLAction_options;
        rclcpp_action::Client<MoveJAction>::SendGoalOptions moveJAction_options;

        rclcpp::CallbackGroup::SharedPtr measurement_callback_group;

        rclcpp::Subscription<custom_srvs_msgs::msg::SpaceRobotState>::SharedPtr space_state_sub;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr tool_ft_sub;

        rclcpp::Client<custom_srvs_msgs::srv::SaveSimulationState>::SharedPtr sim_ctrl_save_state_srv;
        rclcpp::Client<custom_srvs_msgs::srv::LoadSimulationState>::SharedPtr sim_ctrl_load_state_srv;

        std::mutex measurement_mutex;

        MAL online_checker;
        Measurement measurement_state;
        bool has_recieved_first_measurement = false;
        bool do_online_checker = true;
        std::atomic<bool> is_ok = true;

        bool in_test_mode = false;

        std::atomic<MoveState> active_move_state = MoveState::IDLE;

        std::chrono::seconds action_service_wait = 30s;
        std::chrono::seconds service_wait = 3s;

        void space_state_callback(const custom_srvs_msgs::msg::SpaceRobotState::SharedPtr space_state);

        // functions for defining default behaviour
        void configure_moveLAction_options();
        void configure_moveJAction_options();

        void stop();
        void error_log_saved();
        void stop_moveL();
        void stop_moveJ();

        /**
         * Function that is called upon stop, and saves the state. Blocks until the response is received.
         */
        void trigger_save_sim_state();
        /**
         * callback that enables handle to saved simulation state
         */
        void save_sim_state_callback(rclcpp::Client<custom_srvs_msgs::srv::SaveSimulationState>::SharedFuture future_response);

        // send functions
        void send_moveL(geometry_msgs::msg::Pose target_pose, double vel, double acc);
        void send_moveJ(std::vector<double> joint_target, double vel, double acc);

    public:
        explicit MainController(std::string name, std::string conf_file) : Node(name + "_node"), online_checker(name, 16, 15)
        {
            this->open_gripper_srv = this->create_client<EmptySrv>("/open_gripper");
            this->close_gripper_srv = this->create_client<EmptySrv>("/close_gripper");
            this->move_j_client = rclcpp_action::create_client<MoveJAction>(this, "action/move_j");
            this->move_l_client = rclcpp_action::create_client<MoveLAction>(this, "action/move_l");

            this->sim_ctrl_save_state_srv = this->create_client<custom_srvs_msgs::srv::SaveSimulationState>("/save_simulation_state");
            this->sim_ctrl_load_state_srv = this->create_client<custom_srvs_msgs::srv::LoadSimulationState>("/load_simulation_state");

            // Measurement Monitor setup
            measurement_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            auto sub_options = rclcpp::SubscriptionOptions();
            sub_options.callback_group = measurement_callback_group;
            space_state_sub = this->create_subscription<custom_srvs_msgs::msg::SpaceRobotState>("/vispa/space_robot_state", 10, std::bind(&MainController::space_state_callback, this, std::placeholders::_1), sub_options);
            active_move_state = MoveState::IDLE;

            // Configure Online Checker
            online_checker.configure(std::bind(&MainController::stop, this), std::bind(&MainController::trigger_save_sim_state, this), conf_file, std::bind(&MainController::error_log_saved, this));
            online_checker.set_analyse_flags(CartesianSpace::AnalyseFlag::Torque | CartesianSpace::AnalyseFlag::Force, JointSpace::AnalyseFlag::AngularVelocity | JointSpace::AnalyseFlag::Torque);
            online_checker.check_freeZone_constraints_only(true);
            online_checker.enable();

            configure_moveLAction_options();
            configure_moveJAction_options();
        }
        ~MainController();
        /** Move the robot Linearly
         * @param target_pose
         * @param vel default 0.02
         * @param acc default 0.2
         */
        void moveL(geometry_msgs::msg::Pose target_pose, double vel = 0.02, double acc = 0.2, bool free_move = false);
        void moveL(CartesianSpace::Pose target_pose, double vel = 0.02, double acc = 0.2, bool free_move = false);

        bool did_reached_target();

        /** Move the Robot P2P
         * @param joint_target
         * @param vel default 0.02
         * @param acc default 0.2
         */
        void moveJ(std::vector<double> joint_target, double vel = 0.02, double acc = 0.2);

        /** Open gripper on robot
         */
        void open_gripper();

        /**
         * close gripper on robot
         */
        void close_gripper();

        /**
         * Informs about state. Does the system move or not. and which type moveL or moveJ
         */
        MoveState get_moveState();

        bool get_ok();

        /**
         * a demo function that allows for easy switch between MAPLE loop or not.
         * @param do_MAL_loop if true, the system performs online_checks | false results in no online checks
         */
        void with_MAPLE_loop(bool do_MAL_loop = true);

        /**
         * save measurement
         */
        void save_measurements(std::string filename);

        /** flag for initiating the robot program execution */
        bool has_updated_measurement();

        void load_sim_state_srv(std::string path);

        /**
         * @return returns path to error file
         */
        std::string get_error_file();

        /**
         * @return path to environment file
         */
        std::string get_env_file();

        /** Toggles Test mode.
         * test_mode = True, doesn't save logs and files
         * Test_mode = False, saves logs, necessary for the MAPLE-K loop
         */
        void set_test_mode(const bool &test_mode_on = false);

        /**
         * Reset main controller is_ok flag
         */
        void reset();

        ConstraintRelaxer &get_constraint_monitor();
    };

}
#endif // MAINCONTROLLER_HPP