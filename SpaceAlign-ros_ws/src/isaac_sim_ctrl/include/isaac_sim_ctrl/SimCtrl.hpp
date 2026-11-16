#ifndef SIM_CTRL_HPP
#define SIM_CTRL_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <list>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "simulation_interfaces/srv/load_world.hpp"
#include "simulation_interfaces/srv/set_simulation_state.hpp"
#include "simulation_interfaces/srv/get_entities_states.hpp"
#include "simulation_interfaces/msg/entity_state.hpp"
#include "simulation_interfaces/msg/simulation_state.hpp"

#include "custom_srvs_msgs/srv/motion_traj.hpp"
#include "custom_srvs_msgs/srv/ptp_joint_value_target.hpp"
#include "custom_srvs_msgs/srv/save_simulation_state.hpp"
#include "custom_srvs_msgs/srv/load_simulation_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "isaac_sim_ctrl/MoveJointActionHandler.hpp"
#include "isaac_sim_ctrl/MovePoseActionHandler.hpp"


class SimCtrl : public rclcpp::Node
{
    public:
        using LoadWorldSrv = simulation_interfaces::srv::LoadWorld;
        using SetSimulationStateSrv = simulation_interfaces::srv::SetSimulationState;
        using GetEntitiesStatesSrv = simulation_interfaces::srv::GetEntitiesStates;
        using EntityStateMsg = simulation_interfaces::msg::EntityState;
        using EmptySrv = std_srvs::srv::Empty;
        using MotionTrajSrv = custom_srvs_msgs::srv::MotionTraj;
        using PTPJointValueTargetSrv = custom_srvs_msgs::srv::PTPJointValueTarget;
        using SaveSimulationStateSrv = custom_srvs_msgs::srv::SaveSimulationState;
        using LoadSimulationStateSrv = custom_srvs_msgs::srv::LoadSimulationState;
        using SwitchControllerSrv = controller_manager_msgs::srv::SwitchController;
        
        SimCtrl();
        void initialize();

    private:
        void robot_joint_states(sensor_msgs::msg::JointState::UniquePtr msg);
        bool movej(std::vector<double> target_joint_values, double vel_factor, double acc_factor);
        bool movej(geometry_msgs::msg::Pose target_pose, double vel_factor, double acc_factor);
        void move_ptp(const std::shared_ptr<MotionTrajSrv::Request> request, std::shared_ptr<MotionTrajSrv::Response> response);
        void move_ptp_jpos(const std::shared_ptr<PTPJointValueTargetSrv::Request> request, std::shared_ptr<PTPJointValueTargetSrv::Response> response);
        void move_lin(const std::shared_ptr<MotionTrajSrv::Request> request, std::shared_ptr<MotionTrajSrv::Response> response);
        void open_gripper(const std::shared_ptr<EmptySrv::Request>, std::shared_ptr<EmptySrv::Response>);
        void close_gripper(const std::shared_ptr<EmptySrv::Request>, std::shared_ptr<EmptySrv::Response>);
        void geometry_msgs_Pose_to_eigen(const geometry_msgs::msg::Pose &pose, Eigen::Matrix4d &matrix);
        void save_simulation_state(const std::shared_ptr<SaveSimulationStateSrv::Request>, std::shared_ptr<SaveSimulationStateSrv::Response> save_sim_response);
        void load_simulation_state(const std::shared_ptr<LoadSimulationStateSrv::Request> request, std::shared_ptr<LoadSimulationStateSrv::Response> response);
        bool call_set_simulation_state_srv(uint state);
        bool call_load_world_srv(std::string usdfile_uri);
        bool call_switch_controller(bool activate);
        bool wait_for_joint_state(const std::vector<double> &target, std::size_t previous_update_count);

        rclcpp::CallbackGroup::SharedPtr _callback_group;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;

        rclcpp::Client<LoadWorldSrv>::SharedPtr _load_world_client;
        rclcpp::Client<SetSimulationStateSrv>::SharedPtr _set_sim_state_client;
        rclcpp::Client<GetEntitiesStatesSrv>::SharedPtr _get_entities_states_client;
        rclcpp::Client<SwitchControllerSrv>::SharedPtr _switch_controller_client;

        rclcpp::Service<MotionTrajSrv>::SharedPtr _move_ptp_srv;
        rclcpp::Service<PTPJointValueTargetSrv>::SharedPtr _move_ptp_jpos_srv;
        rclcpp::Service<MotionTrajSrv>::SharedPtr _move_lin_srv;

        rclcpp::Service<EmptySrv>::SharedPtr _open_gripper_srv;
        rclcpp::Service<EmptySrv>::SharedPtr _close_gripper_srv;

        rclcpp::Service<SaveSimulationStateSrv>::SharedPtr _save_simulation_state_srv;
        rclcpp::Service<LoadSimulationStateSrv>::SharedPtr _load_simulation_state_srv;

        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> _move_group_vispa;
		
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _robot_joint_states_pub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_commands_reset_pub;
        rclcpp::TimerBase::SharedPtr _robot_joint_states_timer;
        bool _gripper_open;

		MoveJointActionHandler _moveJointActHndl;
		MovePoseActionHandler _movePoseActHndl;

        std::mutex _robot_joint_states_mtx;
        std::vector<double> _robot_joint_states_pos;
        std::size_t _robot_joint_states_update_count{0};
};

#endif // SIM_CTRL_HPP
