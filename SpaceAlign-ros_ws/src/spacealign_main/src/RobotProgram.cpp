#include "spacealign_main/RobotProgram.hpp"

RobotProgram::RobotProgram(const std::string &path, size_t numberOfJoints)
{
    program_name = path;
    number_of_joints = numberOfJoints;
    reset();
}

RobotProgram::RobotProgram(size_t numberOfJoints)
{
    number_of_joints = numberOfJoints;
    reset();
}

RobotProgram::~RobotProgram()
{
    if (program_loop.joinable())
        program_loop.join();
    if (ros_loop.joinable())
        ros_loop.join();
}

void RobotProgram::init(std::shared_ptr<ROS_INTERFACE::MainController> _node)
{
    node = _node;
}
void RobotProgram::reset()
{
    velocity_fraction = 0.02;
    acceleration_fraction = 0.2;
    save_idx = 0;
    action_list.clear();
    save_list.clear();
}

void RobotProgram::switch_case(Action action)
{
    switch (action.state)
    {
    case RobotAction::HOME:
        action_HOME();
        break;
    case RobotAction::IDLE:
        break;
    case RobotAction::MOVEL:
        action_MOVEL(action.position);
        break;
    case RobotAction::MOVEL_FREE:
        action_MOVEL_FREE(action.position);
        break;
    case RobotAction::MOVEJ:
        action_MOVEJ(action.position);
        break;
    case RobotAction::GRASP:
        action_GRASP();
        break;
    case RobotAction::WAIT:
        action_WAIT(action.position);
        break;
    case RobotAction::SAVE:
        action_SAVE();
        break;
    case RobotAction::UPDATE_VEL_ACC:
        action_UPDATE_VEL_ACC(action.position);
        break;
    case RobotAction::RELEASE:
        action_RELEASE();
        break;
    default:
        RCLCPP_ERROR(node->get_logger(), "ERROR: Reached default state");
        break;
    }
}

void RobotProgram::program_execution()
{
    int i = 0;
    Action current_action;
    bool is_error_occurred = false;
    std::chrono::time_point last_error_tp = std::chrono::steady_clock::now() - 10s;
    while (i < (int)(action_list.size()) && is_running)
    {
        // Obtain current action
        current_action = action_list.at(i);

        // Do determined action
        switch_case(current_action);

        // Wait for the action to be completed
        while (rclcpp::ok())
        {
            // error occurs do error handling
            if (!node->get_ok() || node->get_moveState() == ROS_INTERFACE::MoveState::ERROR_MODE)
            {
                if (node->get_moveState() == ROS_INTERFACE::MoveState::STOPPING)
                {
                    RCLCPP_ERROR(node->get_logger(), "Cancellation time out stopping node!");
                    rclcpp::shutdown();
                    return;
                }

                is_error_occurred = true;
                std::string logging_data = "Error: Detected!!!!\n node_ok:\t" + std::to_string(node->get_ok()) + "\t node_moveState:\t" + std::to_string(node->get_moveState());

                RCLCPP_ERROR(node->get_logger(), logging_data.c_str());
                is_running = false;
                break;
            }
            // Action completed, continue to the next one
            else if (node->get_moveState() == ROS_INTERFACE::MoveState::IDLE)
            {
                break;
            }
            // If ok then wait 500ms and check again. System stop is handled on a lower level. see MainController
            else
            {
                this_thread::sleep_for(500ms);
            }
        }
        // check for errors
        if (is_error_occurred == false)
        {
            i++;
        }
        else
        {
            std::chrono::time_point tp_error = std::chrono::steady_clock::now();
            if (tp_error - last_error_tp < 2s)
            {
                RCLCPP_INFO(node->get_logger(), "Consecutive Errors. Error occurred within 2s from the last");
                rclcpp::shutdown();
                return;
            }
            bool is_successful = do_ERROR_RECOVERY(current_action);
            if (is_successful)
            {
                i++;
                // reset driver state
                node->reset();
                // failure handled, no error occurred.
                is_error_occurred = false;
                is_running = true;
                last_error_tp = std::chrono::steady_clock::now();
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "{MAPLE-K}[ERROR_RECOVERY]: Unsuccessful recovery");
                return;
            }
        }
    }
    is_running = false;
    n_successful_completion++;
}

void RobotProgram::ros_execution()
{

    rclcpp::executors::MultiThreadedExecutor executer2;
    executer2.add_node(node);
    RCLCPP_INFO(node->get_logger(), "MultiThreadedExecutor started.");
    try
    {
        is_ros_node_running = true;
        executer2.spin();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "execution exception: %s", e.what());
    }
    rclcpp::shutdown();
    is_ros_node_running = false;
    RCLCPP_INFO(node->get_logger(), "MultiThreadedExecutor Stopped Spinning.");
}

void RobotProgram::execute()
{
    is_running = true;
    ros_loop = std::thread(std::bind(&RobotProgram::ros_execution, this));
    std::cout << "spining ros_loop" << std::endl;
    while (true)
    {
        if (node->has_updated_measurement())
            break;
        this_thread::sleep_for(1s);
    }
    std::cout << "starting program" << std::endl;
    program_execution();

    std::fstream file;

    std::string file_name = (boost::filesystem::path(save_dir).parent_path() / "recovery-stat.csv").string();
    file.open(file_name, std::fstream::in | std::fstream::out | std::fstream::app);
    if (file.is_open())
    {
        file << boost::filesystem::path(save_dir).filename().string() << "," << n_successful_completion << "," << n_recovery << "," << n_recovery_successes << "," << n_recovery_fails << std::endl;
        file.close();
    }
    else
    {
        std::cout << "===== RECOVERY STATS =====" << "\n"
                  << "Folder: \t" << boost::filesystem::path(save_dir).filename().string() << "\n"
                  << " Fail free: \t" << n_successful_completion << "\n"
                  << "# recovery calls: \t" << n_recovery << "\n"
                  << "# recovery success: \t" << n_recovery_successes << "\n"
                  << "# recovery fails: \t" << n_recovery_fails << std::endl;
    }
}

void RobotProgram::add_action(Action a)
{
    action_list.push_back(a);
}

void RobotProgram::add_action(RobotAction task)
{
    Action a;
    a.state = task;
    action_list.push_back(a);
}

void RobotProgram::add_debug_action(RobotAction task, std::string fname)
{
    assert(task == RobotAction::SAVE);
    Action a;
    a.state = task;
    action_list.push_back(a);
    save_list.push_back(fname);
}

void RobotProgram::add_action(RobotAction task, std::vector<double> position)
{
    Action a;
    a.position = position;
    a.state = task;
    action_list.push_back(a);
}

bool RobotProgram::do_ERROR_RECOVERY(Action failed_action)
{
    RCLCPP_INFO(node->get_logger(), "{MAPLE-K}[ERROR_RECOVERY]: Begun");

    n_recovery++;
    // Exhaustive search
    if (true)
    {
        // set test mode to disable overwrite of environment file
        node->set_test_mode(true);

        // Action Planner initiation
        Offline::APLE<RobotAction> aple(program_name, std::bind(&RobotProgram::test_handle, this, std::placeholders::_1), std::bind(&RobotProgram::load_sim_env, this, std::placeholders::_1));

        // Definition of action space
        aple.set_action_space(RobotAction::MOVEL_FREE, RobotAction::MOVEJ, RobotAction::UPDATE_VEL_ACC);
        Offline::Recovery::Plan<RobotAction> recover_plan = aple.get_solution(failed_action.state, node->get_error_file(), node->get_env_file(), node->get_constraint_monitor());

        // disable testing mode
        node->set_test_mode(false);
        aple.execute_mitigation(std::bind(&RobotProgram::recovery_execution_handle, this, std::placeholders::_1), recover_plan, node->get_constraint_monitor());
        // Info
        std::string comment = "{MAPLE-K}[ERROR_RECOVERY]: Ended | \t Found solution: ";
        comment += (node->did_reached_target()) ? "TRUE" : "FALSE";
        RCLCPP_INFO(node->get_logger(), comment.c_str());

        if (node->did_reached_target())
        {
            n_recovery_successes++;
        }
        else
        {
            n_recovery_fails++;
        }
        // check if target is reached
        return node->did_reached_target();
    }
    std::string comment = "{MAPLE-K}[ERROR_RECOVERY]: Ended";
    RCLCPP_INFO(node->get_logger(), comment.c_str());

    return false;
}

void RobotProgram::action_HOME()
{
    std::cout << "action_HOME:\t Started " << std::endl;
    node->moveJ(home_pos, velocity_fraction, acceleration_fraction);
    std::cout << "action_HOME:\t Ended " << std::endl;
}
void RobotProgram::action_IDLE()
{
    std::cout << "action_IDLE:\t " << std::endl;
}

void RobotProgram::action_MOVEL(std::vector<double> position)
{
    CartesianSpace::Pose p;
    std::cout << "action_MOVEL:\t Started" << std::endl;
    if (position.size() != 7)
    {
        throw std::runtime_error("Position needs to be 7 elements translation: x,y,z and rotation w,x,y,z. Input was: " + std::to_string(position.size()));
    }
    p.trans = Eigen::Vector3d::Map(position.data(), 3);
    p.rotation = Eigen::Quaterniond(position.at(3), position.at(4), position.at(5), position.at(6));
    node->moveL(p, velocity_fraction, acceleration_fraction);
    std::cout << "action_MOVEL:\t Ended" << std::endl;
}

void RobotProgram::action_MOVEL_FREE(std::vector<double> position)
{
    CartesianSpace::Pose p;
    std::cout << "action_MOVEL_FREE:\t Started" << std::endl;
    if (position.size() != 7)
    {
        throw std::runtime_error("Position needs to be 7 elements translation: x,y,z and rotation w,x,y,z. Input was: " + std::to_string(position.size()));
    }
    p.trans = Eigen::Vector3d::Map(position.data(), 3);
    p.rotation = Eigen::Quaterniond(position.at(3), position.at(4), position.at(5), position.at(6));
    node->moveL(p, velocity_fraction, acceleration_fraction, true);
    std::cout << "action_MOVEL_FREE:\t Ended" << std::endl;
}

void RobotProgram::action_MOVEJ(std::vector<double> position)
{
    std::cout << "action_MOVEJ:\t Started" << std::endl;
    if (position.size() != number_of_joints)
    {
        throw std::runtime_error("Position needs to be equal to the number of joints. Input was: " + std::to_string(position.size()));
    }
    node->moveJ(position, velocity_fraction, acceleration_fraction);
    std::cout << "action_MOVEJ:\t Ended" << std::endl;
}
void RobotProgram::action_GRASP()
{
    std::cout << "action_GRASP:\t Started" << std::endl;
    node->close_gripper();
    std::cout << "action_GRASP:\t Ended" << std::endl;
}

void RobotProgram::action_RELEASE()
{
    std::cout << "action_RELEASE:\t Started" << std::endl;
    node->open_gripper();
    std::cout << "action_RELEASE:\t Ended" << std::endl;
}

void RobotProgram::action_WAIT(std::vector<double> time_to_wait_ms)
{
    std::cout << "action_WAIT:\t Started" << std::endl;
    this_thread::sleep_for(chrono::milliseconds((int)time_to_wait_ms.at(0)));
    std::cout << "action_WAIT:\t Ended" << std::endl;
}

void RobotProgram::action_SAVE()
{

    // std::string content = "Save_idx: " + std::to_string(save_idx) + "\t Len of saveList: " + std::to_string(save_list.size());

    // RCLCPP_INFO(node->get_logger(), content.c_str());
    if (save_idx < (int)(save_list.size()))
    {
        save_recording(save_list.at(save_idx));
        save_idx++;
    }
}

void RobotProgram::action_UPDATE_VEL_ACC(std::vector<double> param)
{
    std::string content = "Velocity & Acceleration are updated! { ";
    for (double &d : param)
    {
        content += std::to_string(d) + ", ";
    }
    content = content.substr(0, content.size() - 2);
    content += "}";
    RCLCPP_INFO(node->get_logger(), content.c_str());

    assert(param.size() == 2);
    set_velocity_acc(param.at(0), param.at(1));
}

bool RobotProgram::get_is_running()
{
    return is_running;
}

void RobotProgram::do_safety_checks(bool do_checks)
{
    node->with_MAPLE_loop(do_checks);
}

void RobotProgram::set_velocity_acc(double vel, double acc)
{
    assert(vel <= 1);
    assert(vel > 0);
    assert(acc <= 1);
    assert(acc > 0);

    velocity_fraction = vel;
    acceleration_fraction = acc;
}

void RobotProgram::set_save_directory(string _save_dir)
{

    std::chrono::time_point tp_now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(tp_now);
    std::string time_point_str = std::ctime(&t);
    time_point_str.resize(time_point_str.size() - 1);
    save_dir = _save_dir + "/" + time_point_str;
}

void RobotProgram::save_recording(string file)
{
    std::string content = "SAVING FILE: " + file;
    RCLCPP_INFO(node->get_logger(), content.c_str());
    node->save_measurements(save_dir + "/" + file);
}

double RobotProgram::test_handle(const std::vector<Offline::Recovery::Action<RobotAction>> &test_actions)
{
    double err_count = 0.0;
    node->reset();
    for (int i = 0; i < (int)(test_actions.size()); i++)
    {
        Action recover_action;
        recover_action.state = test_actions.at(i).action;
        recover_action.position = test_actions.at(i).target;

        // Restart Main controller if it was shutdown
        if (is_ros_node_running == false)
        {
            if (ros_loop.joinable())
            {
                ros_loop.join();
            }
            ros_loop = std::thread(std::bind(&RobotProgram::ros_execution, this));
        }

        switch_case(recover_action);

        // wait for the action to actually complete before firing the next one
        while (node->get_moveState() != ROS_INTERFACE::MoveState::IDLE)
        {
            if (node->get_moveState() == ROS_INTERFACE::MoveState::ERROR_MODE)
            {
                err_count += 10.0;
                return err_count;
            }
            this_thread::sleep_for(500ms);
        }
    }
    // error_penalty
    return err_count;
}

bool RobotProgram::recovery_execution_handle(const std::vector<Offline::Recovery::Action<RobotAction>> &recovery_actions)
{
    node->reset();
    // Restart Main controller if it was shutdown
    if (is_ros_node_running == false)
    {
        if (ros_loop.joinable())
        {
            ros_loop.join();
        }
        ros_loop = std::thread(std::bind(&RobotProgram::ros_execution, this));
    }

    for (const Offline::Recovery::Action<RobotAction> &rec_action : recovery_actions)
    {
        Action action;
        action.state = rec_action.action;
        action.position = rec_action.target;

        switch_case(action);
        // wait for the action to actually complete before firing the next one
        while (node->get_moveState() != ROS_INTERFACE::MoveState::IDLE)
        {
            if (node->get_moveState() == ROS_INTERFACE::MoveState::ERROR_MODE)
            {
                // If Error => faulty recovery
                return false;
            }
            this_thread::sleep_for(500ms);
        }
    }
    return true;
}

void RobotProgram::load_sim_env(std::string path)
{
    node->load_sim_state_srv(path);
}