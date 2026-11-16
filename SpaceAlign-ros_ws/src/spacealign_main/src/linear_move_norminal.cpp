#ifndef SPACE_ALIGN_MAIN
#define SPACE_ALIGN_MAIN

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <cstdlib>

#include "spacealign_main/MainController.hpp"
#include "spacealign_main/RobotProgram.hpp"

double d2r(double value)
{
    return 3.1415926535897932384626433832795 * value / 180.0;
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    string config_path = string(config_file_path); 
    std::string working_dir = "Test_SPA_test/lin_Nominal";

    std::shared_ptr<ROS_INTERFACE::MainController> node = std::make_shared<ROS_INTERFACE::MainController>("test_linNorm",config_path);    

    RobotProgram program(working_dir, 6);
    program.init(node);
    program.do_safety_checks(true);

    program.set_save_directory(working_dir);


    program.add_action(RobotAction::RELEASE,{});
    
    program.add_action(RobotAction::UPDATE_VEL_ACC, {0.05,0.2});
    program.add_action(RobotAction::HOME,{});
    program.add_action(RobotAction::MOVEJ,{d2r(45),d2r(20),d2r(84),d2r(0),d2r(60),d2r(0)}); 

    program.add_action(RobotAction::MOVEL_FREE, {-1.000, -0.800, 0.260, 0.0, 1.0, 0.0, 0.0});
    program.add_debug_action(RobotAction::SAVE,"Approach");
    
    // Start Grasp
    program.add_action(RobotAction::UPDATE_VEL_ACC, {0.02,0.05});
    program.add_action(RobotAction::MOVEL_FREE, {-1.000, -0.800, 0.060, 0.0, 1.0, 0.0, 0.0});
    program.add_action(RobotAction::GRASP,{});
    program.add_debug_action(RobotAction::SAVE,"Grasp");

    // move with truss

    // Lift truss
    program.add_action(RobotAction::MOVEL_FREE, {-1.000, -0.800, 0.070, 0.0, 1.0, 0.0, 0.0});
    // Linear move
    program.add_action(RobotAction::UPDATE_VEL_ACC, {0.05,0.2});
    program.add_action(RobotAction::MOVEL_FREE, {1.000, -0.800, 0.070, 0.0, 1.0, 0.0, 0.0});
    program.add_debug_action(RobotAction::SAVE,"lin_move");
    
    // retract
    program.add_action(RobotAction::UPDATE_VEL_ACC, {0.05,0.2});
    program.add_action(RobotAction::WAIT,{2000.5});
    program.add_action(RobotAction::RELEASE,{});
    program.add_action(RobotAction::WAIT, {2000,5});
    program.add_action(RobotAction::MOVEL_FREE, {1.000, -0.800, 0.260, 0.0, 1.0, 0.0, 0.0});    
    program.add_action(RobotAction::HOME,{});
    program.add_debug_action(RobotAction::SAVE,"Retract");
    
    // execute robot program
    program.execute();
    
    if(rclcpp::ok() && program.get_is_running())
    {
        std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(node->get_logger(),"Node is shutting down");

    rclcpp::shutdown();

    return 0;
}

#endif