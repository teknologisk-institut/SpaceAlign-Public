#include "spacealign_main/MainController.hpp"

namespace ROS_INTERFACE
{

    MainController::~MainController()
    {
    }

    void MainController::moveL(geometry_msgs::msg::Pose target_pose, double vel, double acc, bool free_move)
    {
        CartesianSpace::Pose pose;
        pose.rotation = Eigen::Quaterniond(target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
        pose.trans = Eigen::Vector3d(target_pose.position.x, target_pose.position.y, target_pose.position.z);
        if (do_online_checker)
        {
            online_checker.disable();
            online_checker.set_cs_target(pose, vel, acc);
            online_checker.check_freeZone_constraints_only(free_move);
            online_checker.enable();
        }
        reached_target = false;
        active_move_state = MoveState::moveL;
        send_moveL(target_pose, vel, acc);
    }

    void MainController::moveL(CartesianSpace::Pose target_pose, double vel, double acc, bool free_move)
    {
        if (do_online_checker)
        {
            online_checker.disable();
            online_checker.set_cs_target(target_pose, vel, acc);
            online_checker.check_freeZone_constraints_only(free_move);
            online_checker.enable();
        }
        reached_target = false;
        active_move_state = MoveState::moveL;
        geometry_msgs::msg::Pose pose;
        pose.position.x = target_pose.trans.x();
        pose.position.y = target_pose.trans.y();
        pose.position.z = target_pose.trans.z();
        pose.orientation.w = target_pose.rotation.w();
        pose.orientation.x = target_pose.rotation.x();
        pose.orientation.y = target_pose.rotation.y();
        pose.orientation.z = target_pose.rotation.z();
        std::string position = "|MOVE L| TO POSITION:\t" + std::to_string(pose.position.x) + ",  " + std::to_string(pose.position.y) + ",  " + std::to_string(pose.position.z);
        RCLCPP_INFO(this->get_logger(), position.c_str());
        send_moveL(pose, vel, acc);
    }

    void MainController::stop_moveL()
    {
        move_l_client->async_cancel_all_goals();
        RCLCPP_INFO(this->get_logger(), "Stop_Func: Cancelled all MoveL goals");
        return;
    }

    void MainController::stop_moveJ()
    {
        move_j_client->async_cancel_all_goals();
        RCLCPP_INFO(this->get_logger(), "Stop_Func: Cancelled all MoveJ goals");
        return;
    }

    void MainController::stop()
    {
        MoveState prev = active_move_state;
        active_move_state = MoveState::STOPPING;
        bool cancel_sent = false;
        switch (prev)
        {
        case MoveState::moveJ:
            stop_moveJ();
            cancel_sent = true;
            break;
        case MoveState::moveL:
            stop_moveL();
            cancel_sent = true;
            break;
        default:
            // No motion in progress, so no result callback will ever confirm the stop
            active_move_state = MoveState::STOPPED;
            break;
        }

        if (cancel_sent)
        {
            // Wait for the result callback to confirm the goal ended (STOPPED, or IDLE if it finished before the cancel)
            std::chrono::time_point deadline = std::chrono::steady_clock::now() + 5s;
            while (active_move_state == MoveState::STOPPING && std::chrono::steady_clock::now() < deadline)
            {
                std::this_thread::sleep_for(10ms);
            }
            if (active_move_state == MoveState::STOPPING)
            {
                RCLCPP_ERROR(this->get_logger(), "Timed out while waiting for stop confirmation");
                return;
            }
        }

        if (in_test_mode == false)
        {
            trigger_save_sim_state();
        }


        return;
    }

    void MainController::error_log_saved()
    {
        is_ok = false;
        active_move_state = MoveState::ERROR_MODE;
    }

    void MainController::trigger_save_sim_state()
    {
        while (!sim_ctrl_save_state_srv->wait_for_service(service_wait * 2))
        {
            RCLCPP_INFO(this->get_logger(), "'save_simulation_state' - Service not available");
        }
        custom_srvs_msgs::srv::SaveSimulationState::Request::SharedPtr req = std::make_shared<custom_srvs_msgs::srv::SaveSimulationState::Request>();

        auto future_response = sim_ctrl_save_state_srv->async_send_request(req).future.share();

        // block until the response arrives so env_knowledge_file is set before callers observe ERROR_MODE
        std::future_status status;
        do
        {
            status = future_response.wait_for(std::chrono::milliseconds(10));
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Node shut down while waiting for '/save_simulation_state'.");
                return;
            }
        } while (status != std::future_status::ready);

        save_sim_state_callback(future_response);
    }

    void MainController::save_sim_state_callback(rclcpp::Client<custom_srvs_msgs::srv::SaveSimulationState>::SharedFuture future_response)
    {
        auto response = future_response.get();
        string file_name = response->usda_file;
        online_checker.set_env_knowledge_file(file_name);
    }

    void MainController::load_sim_state_srv(std::string path)
    {
        while (!sim_ctrl_load_state_srv->wait_for_service(service_wait))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_ERROR(this->get_logger(), "'/load_simulation_state' - Service not available, waiting again...");
        }
        custom_srvs_msgs::srv::LoadSimulationState::Request::SharedPtr req = std::make_shared<custom_srvs_msgs::srv::LoadSimulationState::Request>();

        std::string entry = "Loading: '" + path + "'";
        RCLCPP_INFO(this->get_logger(), entry.c_str());
        // req->set__usda_file(online_checker.get_env_knowledge_file());

        req->set__usda_file(path);

        for (int i = 0; i < 3; i++)
        {
            auto future_response = sim_ctrl_load_state_srv->async_send_request(req);

            // Wait for the result
            std::future_status status;
            do
            {
                status = future_response.wait_for(std::chrono::milliseconds(10));
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Node shut down during wait for '/load_simulation_state'.");
                    return;
                }
            } while (status != std::future_status::ready);

            auto response = future_response.get();
            if (response->ok)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully loaded simulation state: '%s'", path.c_str());
                return;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load simulation state: '%s'", path.c_str());
            }
        }
    }

    bool MainController::get_ok()
    {
        return is_ok;
    }

    void MainController::moveJ(std::vector<double> joint_target, double vel, double acc)
    {
        if (do_online_checker)
        {
            online_checker.disable();
            // enable monitor
            online_checker.check_freeZone_constraints_only(true);
            online_checker.set_js_target(joint_target, vel, acc);
            online_checker.enable();
        }
        std::string position = "|MOVE J| TO POSITION:\t";
        for (double &d : joint_target)
        {
            position += "  " + std::to_string(d) + ",";
        }

        RCLCPP_INFO(this->get_logger(), position.c_str());
        reached_target = false;
        active_move_state = MoveState::moveJ;
        send_moveJ(joint_target, vel, acc);
    }

    // Private

    void MainController::send_moveL(geometry_msgs::msg::Pose target_pose, double vel, double acc)
    {

        while (!this->move_l_client->wait_for_action_server(action_service_wait))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server MOVE_L '/action/move_l' not available after waiting");
        }

        MoveLAction::Goal goal_msg;
        goal_msg.target_poses.push_back(target_pose);
        goal_msg.vel = vel;
        goal_msg.acc = acc;

        RCLCPP_INFO(this->get_logger(), "Sending goal");
        move_l_client->async_send_goal(goal_msg, moveLAction_options);
    }

    void MainController::send_moveJ(std::vector<double> joint_target, double vel, double acc)
    {

        while (!move_j_client->wait_for_action_server(action_service_wait))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server MOVE_J '/action/move_j' not available after waiting");
        }

        MoveJAction::Goal goal_msg;
        goal_msg.joint_target = joint_target;
        goal_msg.vel = vel;
        goal_msg.acc = acc;
        RCLCPP_INFO(this->get_logger(), "Sending Joint goal");

        auto goal_handle = move_j_client->async_send_goal(goal_msg, moveJAction_options);
    }

    void MainController::configure_moveLAction_options()
    {
        moveLAction_options.goal_response_callback = [this](const GoalHandleMoveLAction::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                return;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                return;
            }
        };

        moveLAction_options.feedback_callback = [this](GoalHandleMoveLAction::SharedPtr, const std::shared_ptr<const MoveLAction::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->execution_state.c_str());
        };

        moveLAction_options.result_callback = [this](const GoalHandleMoveLAction::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                reached_target = true;
                if (active_move_state == MoveState::moveL)
                    active_move_state = IDLE;
                else if(active_move_state == MoveState::STOPPING)
                    active_move_state = STOPPED;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                // Confirm a commanded stop; a spontaneous abort is an error
                active_move_state = (active_move_state == MoveState::STOPPING) ? MoveState::STOPPED : MoveState::ERROR_MODE;
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                active_move_state = (active_move_state == MoveState::STOPPING) ? MoveState::STOPPED : MoveState::ERROR_MODE;
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                active_move_state = ERROR_MODE;
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
            }
            std::stringstream ss;
            ss << "[" << (result.result->ok ? "SUCCEEDED" : "FAILED") << "] Result received: " << result.result->status;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            return;
        };
    }

    void MainController::configure_moveJAction_options()
    {
        moveJAction_options.goal_response_callback = [this](const GoalHandleMoveJAction::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                return;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                return;
            }
        };

        moveJAction_options.feedback_callback = [this](GoalHandleMoveJAction::SharedPtr, const std::shared_ptr<const MoveJAction::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "Feedback: %s", feedback->execution_state.c_str());
        };

        moveJAction_options.result_callback = [this](const GoalHandleMoveJAction::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                reached_target = true;
                if (active_move_state == MoveState::moveJ )
                    active_move_state = IDLE;
                else if(active_move_state == MoveState::STOPPING)
                    active_move_state = STOPPED;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                // Confirm a commanded stop; a spontaneous abort is an error
                active_move_state = (active_move_state == MoveState::STOPPING) ? MoveState::STOPPED : MoveState::ERROR_MODE;
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                active_move_state = (active_move_state == MoveState::STOPPING) ? MoveState::STOPPED : MoveState::ERROR_MODE;
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                active_move_state = ERROR_MODE;
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
            }
            std::stringstream ss;
            ss << "[" << (result.result->ok ? "SUCCEEDED" : "FAILED") << "] Result received: " << result.result->status;
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            return;
        };
    }

    void MainController::open_gripper()
    {
        while (!open_gripper_srv->wait_for_service(service_wait))
        {
            RCLCPP_INFO(this->get_logger(), "Open gripper service not available, waiting again...");
            this_thread::sleep_for(10ms);
        }
        EmptySrv::Request::SharedPtr req = std::make_shared<EmptySrv::Request>();
        auto handle = open_gripper_srv->async_send_request(req);
    }

    void MainController::close_gripper()
    {
        while (!close_gripper_srv->wait_for_service(service_wait))
        {
            RCLCPP_INFO(this->get_logger(), "Close gripper service not available, waiting again...");
        }
        EmptySrv::Request::SharedPtr req = std::make_shared<EmptySrv::Request>();
        auto handle = close_gripper_srv->async_send_request(req);
    }

    MoveState MainController::get_moveState()
    {
        return active_move_state;
    }

    void MainController::space_state_callback(const custom_srvs_msgs::msg::SpaceRobotState::SharedPtr message)
    {

        std::lock_guard<std::mutex> lock(measurement_mutex);
        has_recieved_first_measurement = true;
        std::vector<double> force, torque;
        force.push_back(message->tcp_force_torque.force.x);
        force.push_back(message->tcp_force_torque.force.y);
        force.push_back(message->tcp_force_torque.force.z);
        torque.push_back(message->tcp_force_torque.torque.x);
        torque.push_back(message->tcp_force_torque.torque.y);
        torque.push_back(message->tcp_force_torque.torque.z);

        measurement_state.set_tcp_ft(force, torque);

        measurement_state.set_time_stamp(std::chrono::system_clock::now());

        // TCP_pose
        CartesianSpace::Pose tcp_pose;
        tcp_pose.trans = Eigen::Vector3d(
            message->tcp_pose.position.x,
            message->tcp_pose.position.y,
            message->tcp_pose.position.z);
        tcp_pose.rotation = Eigen::Quaterniond(
            message->tcp_pose.orientation.w,
            message->tcp_pose.orientation.x,
            message->tcp_pose.orientation.y,
            message->tcp_pose.orientation.z);

        measurement_state.set_tcp_pose(tcp_pose);

        // Copy only the first 6 elements for a 6 dof robot message contains 8 elements as 2 gripper fingers is in the actuation tree
        std::vector<double> j_torque(message->joint_state.effort.begin(), message->joint_state.effort.begin() + 6);
        std::vector<double> j_ang_pos(message->joint_state.position.begin(), message->joint_state.position.begin() + 6);
        std::vector<double> j_ang_vel(message->joint_state.velocity.begin(), message->joint_state.velocity.begin() + 6);

        measurement_state.set_joint_torque(j_torque);
        measurement_state.set_joint_angles(j_ang_pos);
        measurement_state.set_joint_ang_vel(j_ang_vel);
        if (do_online_checker)
        {
            // RCLCPP_INFO(this->get_logger(),"MEASUREMENT RECEIVED");
            online_checker.update_measurement(std::make_shared<Measurement>(measurement_state));
        }
    }

    void MainController::with_MAPLE_loop(bool do_mal_loop)
    {
        do_online_checker = do_mal_loop;
    }

    void MainController::save_measurements(std::string filename)
    {
        online_checker.save(filename);
    }

    bool MainController::has_updated_measurement()
    {
        return has_recieved_first_measurement;
    }

    std::string MainController::get_error_file()
    {
        return online_checker.get_name();
    }

    std::string MainController::get_env_file()
    {
        return online_checker.get_env_knowledge_file();
    }

    void MainController::reset()
    {
        is_ok = true;
        active_move_state = MoveState::IDLE;
    }

    void MainController::set_test_mode(const bool &test_mode)
    {
        in_test_mode = test_mode;
    }

    bool MainController::did_reached_target()
    {
        return reached_target;
    }

    ConstraintRelaxer &MainController::get_constraint_monitor()
    {
        return online_checker;
    }

} // ROS_INTERFACE