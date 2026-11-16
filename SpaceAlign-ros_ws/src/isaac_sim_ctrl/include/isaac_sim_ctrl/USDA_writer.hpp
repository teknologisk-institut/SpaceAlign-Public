#ifndef USDA_WRITER_HPP
#define USDA_WRITER_HPP

#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include "simulation_interfaces/srv/get_entities_states.hpp"


class USDA_writer
{
    using GetEntitiesStatesSrv = simulation_interfaces::srv::GetEntitiesStates;

public:
    static std::string write(const std::string &path, const std::vector<double> &joint_pos, const std::shared_ptr<GetEntitiesStatesSrv::Response> &response);

private:
    static void write_main_file(const std::string &filename, const std::vector<double> &joint_pos, const std::string &winch_usda_file);
    static void write_winch_file(const std::string &filename, const std::vector<Eigen::Matrix4d> &mat_capsules, const Eigen::Matrix4d &mat_bodies, const Eigen::Matrix4d &mat_AttachToolAdaptor, const Eigen::Matrix4d &mat_CylinderGeo);
    static Eigen::Matrix4d geometry_msgs_Pose_to_eigen(const geometry_msgs::msg::Pose &pose);
    static void write_capsule(std::ofstream &out, int capsule_num, const Eigen::Matrix4d &mat);
    static void write_capsule_start(std::ofstream &out);
    static void write_joint(std::ofstream &out, int joint_num, bool start_joint=false);
    static void write_winch_file_start(std::ofstream &out, const Eigen::Matrix4d &mat_AttachToolAdaptor);
    static void write_winch_file_end(std::ofstream &out, const Eigen::Matrix4d &mat_CylinderGeo);

    static void write_main_file_start(std::ofstream &out);
    static void write_main_file_end(std::ofstream &out, double slider_1_pos, double slider_2_pos, const std::string &winch_usda_file);

    static void write_json_jointstate(const std::string &filename, const std::vector<double> &joint_pos);
};

#endif // USDA_WRITER_HPP
