#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <mutex>

#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "custom_srvs_msgs/action/move_pose.hpp"
#include "custom_srvs_msgs/srv/load_simulation_state.hpp"

#include "matplotlibcpp.h"

using MoveLAction = custom_srvs_msgs::action::MovePose;
using GoalHandleMoveLAction = rclcpp_action::ClientGoalHandle<MoveLAction>;
using LoadSimulationStateSrv = custom_srvs_msgs::srv::LoadSimulationState;

using namespace std::chrono_literals;
namespace plt = matplotlibcpp;


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

class RobeRollOutTestClass : public rclcpp::Node
{
public:

    explicit RobeRollOutTestClass()
     : Node("robe_roll_out_test"),
       _worker_running(true),
       _log_vispa_tool_ft(false)
    {
        _move_l_client = rclcpp_action::create_client<MoveLAction>(this, "action/move_l");
        _load_simulation_state_srv = this->create_client<LoadSimulationStateSrv>("/load_simulation_state");

        _vispa_tool_ft_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>("/vispa/tool_ft", 10, std::bind(&RobeRollOutTestClass::vispa_tool_ft_callback, this, std::placeholders::_1));

        _worker_thread = std::thread(&RobeRollOutTestClass::worker_loop, this);
    }

    ~RobeRollOutTestClass()
    {
        _worker_running = false;
        if (_worker_thread.joinable())
            _worker_thread.join();
    }

private:
    void vispa_tool_ft_callback(geometry_msgs::msg::WrenchStamped::UniquePtr msg)
    {
        std::lock_guard<std::mutex> lock(_vispa_tool_ft_log_mutex);
        if(_log_vispa_tool_ft)
        {
            _vispa_tool_ft_log.push_back(*msg);
        }
    }

    void worker_loop()
    {
        double vel = 1.0;
        double acc = 0.2;
        while (acc < 1.1 && rclcpp::ok() && _worker_running)
        {
            RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");
            RCLCPP_INFO(this->get_logger(), "Testing with vel=%lf and acc=%lf", vel, acc);
            RCLCPP_INFO(this->get_logger(), "---");
            RCLCPP_INFO(this->get_logger(), "Loading simualtion");
            this->load_sim();

            RCLCPP_INFO(this->get_logger(), "Sleeping...");
            std::this_thread::sleep_for(1000ms);

            _log_vispa_tool_ft = true;
            RCLCPP_INFO(this->get_logger(), "MoveL");
            MoveLGoalMsg goal_msg = MoveLGoalMsg(vel, acc);
            goal_msg.add_pose(-1.000, -0.800, 0.070, 0.0, 1.0, 0.0, 0.0);
            goal_msg.add_pose( 1.000, -0.800, 0.070, 0.0, 1.0, 0.0, 0.0);
            this->send_movel_goal(goal_msg);
            _log_vispa_tool_ft = false;
            std::this_thread::sleep_for(1000ms);
            RCLCPP_INFO(this->get_logger(), "------------------------------------------------------");

            make_plot(vel, acc);
            {
                std::lock_guard<std::mutex> lock(_vispa_tool_ft_log_mutex);
                _vispa_tool_ft_log.clear();
            }

            acc += 0.2;
        }

        if (rclcpp::ok() && _worker_running)
        {
            rclcpp::shutdown(); 
        }
    }

    void make_plot(double vel, double acc)
    {
        std::vector<geometry_msgs::msg::WrenchStamped> log_snapshot;
        {
            std::lock_guard<std::mutex> lock(_vispa_tool_ft_log_mutex);
            log_snapshot = _vispa_tool_ft_log;
        }

        std::vector<double> t;
        std::vector<double> force_x, force_y, force_z;
        std::vector<double> torque_x, torque_y, torque_z;
        std::vector<geometry_msgs::msg::WrenchStamped>::iterator it = log_snapshot.begin();
        rclcpp::Time msg_time(it->header.stamp);
        double start_t = msg_time.seconds();
        for(; it!=log_snapshot.end(); it++)
        {
            rclcpp::Time msg_time(it->header.stamp);

            t.push_back(msg_time.seconds() - start_t);
            force_x.push_back(it->wrench.force.x);
            force_y.push_back(it->wrench.force.y);
            force_z.push_back(it->wrench.force.z);
            torque_x.push_back(it->wrench.torque.x);
            torque_y.push_back(it->wrench.torque.y);
            torque_z.push_back(it->wrench.torque.z);
        }
        RCLCPP_INFO(this->get_logger(), "Making plots");

        const long nrows=2, ncols=1;
        std::stringstream ss;
        ss << "Tool F/T vel=" << (100*vel) << "% acc=" << (100*acc) << "%";
        plt::suptitle(ss.str());

        RCLCPP_INFO(this->get_logger(), "Making plot 1");
        plt::subplot2grid(nrows, ncols, 0, 0);
        plt::named_plot("F_x", t, force_x);
        plt::named_plot("F_y", t, force_y);
        plt::named_plot("F_z", t, force_z);
        plt::xlabel("Time [s]");
        plt::ylabel("Tool force [N]");
        plt::grid(true);
        plt::legend();

        RCLCPP_INFO(this->get_logger(), "Making plot 2");
        plt::subplot2grid(nrows, ncols, 1, 0);
        plt::named_plot("T_x", t, torque_x);
        plt::named_plot("T_y", t, torque_y);
        plt::named_plot("T_z", t, torque_z);
        plt::xlabel("Time [s]");
        plt::ylabel("Tool torque [Nm]");
        plt::grid(true);
        plt::legend();

        plt::subplots_adjust(std::map<std::string, double>{{"hspace", 0.4}});

        // plt::show();
        RCLCPP_INFO(this->get_logger(), "Saving plot");
        ss.str("");
        ss << "tool_ft_vel" << vel << "_acc" << acc << ".png";
        std::cout << ss.str() << std::endl;
        plt::save(ss.str(), 300);
        RCLCPP_INFO(this->get_logger(), "Closing plot");
        plt::close();
    }

    bool send_movel_goal(const MoveLAction::Goal &goal_msg)
    {
        RCLCPP_INFO(this->get_logger(), "send_movel_goal");
        using namespace std::placeholders;

        if (!this->_move_l_client->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto send_goal_future  = _move_l_client->async_send_goal(goal_msg);      
        if (send_goal_future.wait_for(5s) != std::future_status::ready)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal send request timed out!");
            return false;
        }

        auto goal_handle = send_goal_future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }

        auto result_future = _move_l_client->async_get_result(goal_handle);
        while (rclcpp::ok() && _worker_running)
        {
            if (result_future.wait_for(100ms) == std::future_status::ready)
                break;
        }

        if (!_worker_running)
        {
            RCLCPP_WARN(this->get_logger(), "Action execution canceled because node is shutting down.");
            _move_l_client->async_cancel_goal(goal_handle);
            return false;
        }

        auto wrapped_result = result_future.get();
        if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_WARN(this->get_logger(), "Action did not succeed");
            return false;
        }

        return true;
    }

    bool load_sim()
    {
        while (!_load_simulation_state_srv->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<LoadSimulationStateSrv::Request>();
        request->usda_file = "test1__space_align_vispa_workcell.usda";
        auto future_result = _load_simulation_state_srv->async_send_request(request);
        
        // Wait for the result.
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

        return true;
    }

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr _vispa_tool_ft_sub;
    rclcpp_action::Client<MoveLAction>::SharedPtr _move_l_client;
    rclcpp::Client<LoadSimulationStateSrv>::SharedPtr _load_simulation_state_srv;
    std::thread _worker_thread;
    std::atomic<bool> _worker_running;
    std::atomic<bool> _log_vispa_tool_ft;
    std::mutex _vispa_tool_ft_log_mutex;
    std::vector<geometry_msgs::msg::WrenchStamped> _vispa_tool_ft_log;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobeRollOutTestClass>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}


/*#define _USE_MATH_DEFINES
#include <cmath>
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

int main()
{
    int n = 500;
    std::vector<double> t(n);
    std::vector<double> force_x(n), force_y(n), force_z(n);
    std::vector<double> torque_x(n), torque_y(n), torque_z(n);
    for(int i=0; i<n; ++i)
    {
        t[i] = i;
        force_x[i] = sin(2*M_PI*(i+10)/500.0);
        force_y[i] = sin(2*M_PI*(i+20)/500.0);
        force_z[i] = sin(2*M_PI*(i+30)/500.0);
        torque_x[i] = cos(2*M_PI*(i+10)/500.0);
        torque_y[i] = cos(2*M_PI*(i+20)/500.0);
        torque_z[i] = cos(2*M_PI*(i+30)/500.0);
    }

    const long nrows=2, ncols=1;
    plt::suptitle("Tool F/T");
 
    plt::subplot2grid(nrows, ncols, 0, 0);
    plt::named_plot("F_x", t, force_x);
    plt::named_plot("F_y", t, force_y);
    plt::named_plot("F_z", t, force_z);
    // plt::title("Tool force");
    plt::xlabel("Time [s]");
    plt::ylabel("Tool force [N]");
    plt::grid(true);
    plt::legend();

    plt::subplot2grid(nrows, ncols, 1, 0);
    plt::named_plot("T_x", t, torque_x);
    plt::named_plot("T_y", t, torque_y);
    plt::named_plot("T_z", t, torque_z);
    // plt::title("Tool torque");
    plt::xlabel("Time [s]");
    plt::ylabel("Tool torque [Nm]");
    plt::grid(true);
    plt::legend();

    // std::map<std::string, double> keywords{{"hspace", 0.6}};
    plt::subplots_adjust(std::map<std::string, double>{{"hspace", 0.4}});

    // Show plots
	// plt::show();
    plt::save("test.png", 300);
    plt::close();
}
*/