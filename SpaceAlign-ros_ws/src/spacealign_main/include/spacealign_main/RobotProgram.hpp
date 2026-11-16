#ifndef ROBOTPROGRAM_HPP
#define ROBOTPROGRAM_HPP

#include <memory>
#include <vector>
#include <chrono>
#include <ctime>
#include <thread>
#include <stdexcept>

#include <MAPLE_K-1.0/Offline/APLE.hpp>
#include <MAPLE_K-1.0/Offline/RecoveryStrategy.hpp>

#include <boost/filesystem.hpp>

#include <rclcpp/rclcpp.hpp>

#include "MainController.hpp"

enum RobotAction
{
    HOME = 0,
    IDLE,
    MOVEL,
    MOVEL_FREE,
    MOVEJ,
    GRASP,   // Grasp with gripper
    RELEASE, // Release with gripper
    SAVE,
    UPDATE_VEL_ACC,
    WAIT
};

struct Action
{
    RobotAction state = RobotAction::IDLE;
    std::vector<double> position;
};

class RobotProgram
{
private:
    size_t number_of_joints = 6;
    std::string program_name = "";
    int n_successful_completion = 0;
    int n_recovery = 0;
    int n_recovery_successes = 0;
    int n_recovery_fails = 0;

    std::shared_ptr<ROS_INTERFACE::MainController> node;
    std::vector<double> home_pos = {d2r(50.0), d2r(-20.0), d2r(110.0), 0.0, d2r(90.0), 0.0};
    std::vector<Action> action_list;

    int save_idx = 0;
    std::vector<std::string> save_list;
    std::string save_dir = "";
    bool do_saves = false;

    double velocity_fraction = 0.02;
    double acceleration_fraction = 0.2;

    bool is_running = false;
    bool is_ros_node_running = false;

    double d2r(double value)
    {
        return 3.1415926535897932384626433832795 * value / 180.0;
    }

    std::thread ros_loop;
    std::thread program_loop;

    bool halt_on_action = false;

    void switch_case(Action action);

    void program_execution();
    void ros_execution();
    /**
     * Contrians reset functionality of program
     */
    void reset();

    /** save recording to file */
    void save_recording(string file);

    double test_handle(const std::vector<Offline::Recovery::Action<RobotAction>> &test_actions);
    bool recovery_execution_handle(const std::vector<Offline::Recovery::Action<RobotAction>> &recovery_actions);
    void load_sim_env(std::string file_name);

protected:
    bool do_ERROR_RECOVERY(Action failed_action);
    void action_HOME();
    /**
     * Time to wait in ms
     * @param time in ms {500} => 0,5 s
     */
    void action_WAIT(std::vector<double> time);
    void action_IDLE();
    void action_MOVEL(std::vector<double> position);
    void action_MOVEL_FREE(std::vector<double> position);
    void action_MOVEJ(std::vector<double> position);
    void action_GRASP();
    void action_SAVE();
    void action_UPDATE_VEL_ACC(std::vector<double> param);
    void action_RELEASE();

public:
    RobotProgram(size_t numberOFJoints = 6);
    RobotProgram(const std::string &program_name, size_t numberOFJoints = 6);
    void init(std::shared_ptr<ROS_INTERFACE::MainController> _node);

    void set_program_name(const std::string &program_name);

    void do_safety_checks(bool do_checks = true);

    void add_action(RobotAction task);

    void add_action(Action a);

    void add_debug_action(RobotAction task, std::string fname);

    void add_action(RobotAction task, std::vector<double> position);

    /**
     * setting velocity and acceleration.
     * vel % of max velocity default 0.02
     * acc % of max acceleration default 0.2
     */
    void set_velocity_acc(double vel = 0.02, double acc = 0.2);

    bool get_is_running();
    void execute();

    void set_save_directory(string path);
    ~RobotProgram();
};

#endif // ROBOTPROGRAM_HPP