#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP

/** @file Measurement.hpp
 *  @brief Cartesian and joint-space measurement values.
 */
#include <memory>
#include <vector>
#include <chrono>
#include <cstdint>
#include <stdexcept>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ctime>

namespace CartesianSpace
{
    /** Cartesian position/orientation or derivative represented as a pose. */
    struct Pose
    {
        /** Translation component. */
        Eigen::Vector3d trans = Eigen::Vector3d::Zero();
        /** Orientation quaternion. */
        Eigen::Quaternion<double> rotation = Eigen::Quaternion<double>::Identity();
        /** Whether the pose contains a valid value. */
        bool available = false;

        /**
         * Converts the pose to a seven-element vector.
         * @return Values ordered [tx, ty, tz, qw, qx, qy, qz].
         */
        explicit operator std::vector<double>() const
        {
            return {
                trans.x(), trans.y(), trans.z(),
                rotation.w(), rotation.x(), rotation.y(), rotation.z()};
        }

        /**
         * Creates a pose from a seven-element vector.
         * @param values Values ordered [tx, ty, tz, qw, qx, qy, qz].
         * @throws std::invalid_argument If values does not contain exactly seven elements.
         */
        static Pose from_vector(const std::vector<double> &values)
        {
            if (values.size() != 7)
            {
                throw std::invalid_argument("Pose vector must contain 7 values: [tx, ty, tz, qw, qx, qy, qz].");
            }

            Pose pose;
            pose.trans << values[0], values[1], values[2];
            pose.rotation = Eigen::Quaternion<double>(values[3], values[4], values[5], values[6]);
            pose.available = true;
            return pose;
        }
    };
}

/** Holds one timestamped set of Cartesian and joint-space measurements. */
class Measurement
{
private:
    CartesianSpace::Pose tcp_pose;
    CartesianSpace::Pose tcp_vel;
    CartesianSpace::Pose tcp_acc;
    Eigen::Vector<double, 6> tcp_ft = Eigen::Vector<double, 6>::Zero();
    Eigen::VectorXd joint_angles;
    Eigen::VectorXd joint_ang_vel;
    Eigen::VectorXd joint_ang_acc;
    Eigen::VectorXd joint_torque;
    Eigen::VectorXd joint_temp;
    Eigen::VectorXd joint_power;
    std::time_t timestamp = 0;
    std::int64_t timestamp_ms_epoch = 0;

    bool free = false;
    // Cartesian Space

    bool is_available_tcp_pose = false;
    bool is_available_tcp_vel = false;
    bool is_available_tcp_acc = false;
    bool is_available_tcp_ft = false;

    // Joint space
    bool is_available_joint_angles = false;
    bool is_available_joint_ang_vel = false;
    bool is_available_joint_ang_acc = false;
    bool is_available_joint_torque = false;

    bool is_available_joint_power = false;
    bool is_available_joint_temp = false;
    bool is_available_timestamp = false;

    void update_availability();

public:
    /** Constructs an empty measurement with all fields unavailable. */
    Measurement();

    /**
     * Copies the values and availability state from another measurement.
     * @param meas Measurement to copy.
     */
    Measurement(std::shared_ptr<const Measurement> meas);

    /**
     * Sets availability for joint angles, velocities, and accelerations.
     * @param available Whether those fields should be marked available.
     * @param include_force Whether joint torque should also be marked available.
     */
    void set_availability_jointSpace(bool available, bool include_force = false);

    /**
     * Sets availability for TCP pose, velocity, and acceleration.
     * @param available Whether those fields should be marked available.
     * @param include_force Whether TCP force/torque should also be marked available.
     */
    void set_availability_cartSpace(bool available, bool include_force = false);

    /**
     * Marks joint-power data as available or unavailable.
     * @param available Whether joint-power data is available.
     */
    void set_availability_power(bool available);

    /**
     * Marks joint-temperature data as available or unavailable.
     * @param available Whether joint-temperature data is available.
     */
    void set_availability_temp(bool available);

    /**
     * Sets the TCP pose and marks it available.
     * @param _tcp_pose TCP position and orientation.
     */
    void set_tcp_pose(const CartesianSpace::Pose &_tcp_pose);
    /**
     * Sets the TCP velocity and marks it available.
     * @param _tcp_vel TCP linear and angular velocity.
     */
    void set_tcp_vel(const CartesianSpace::Pose &_tcp_vel);
    /**
     * Sets the TCP acceleration and marks it available.
     * @param _tcp_acc TCP linear and angular acceleration.
     */
    void set_tcp_acc(const CartesianSpace::Pose &_tcp_acc);
    /**
     * Sets the six-axis TCP force/torque vector and marks it available.
     * @param _tcp_ft Six force/torque components.
     */
    void set_tcp_ft(const Eigen::Vector<double, 6> &_tcp_ft);
    /**
     * Sets TCP force and torque components and marks them available.
     * @param force Three force components ordered x, y, z.
     * @param torque Three torque components ordered x, y, z.
     * @throws std::out_of_range If either vector has fewer than three elements.
     */
    void set_tcp_ft(const std::vector<double> &force, const std::vector<double> &torque);

    /**
     * Sets joint angles and marks them available.
     * @param _joint_angles Joint angles, one value per joint.
     */
    void set_joint_angles(const Eigen::VectorXd &_joint_angles);
    /**
     * Sets joint angles from a standard vector and marks them available.
     * @param _joint_angles Joint angles, one value per joint.
     */
    void set_joint_angles(const std::vector<double> &_joint_angles);
    /**
     * Sets joint angular velocity and marks it available.
     * @param _joint_ang_vel Angular velocity, one value per joint.
     */
    void set_joint_ang_vel(const Eigen::VectorXd &_joint_ang_vel);
    /**
     * Sets joint angular velocity from a standard vector and marks it available.
     * @param _joint_ang_vel Angular velocity, one value per joint.
     */
    void set_joint_ang_vel(const std::vector<double> &_joint_ang_vel);
    /**
     * Sets joint angular acceleration and marks it available.
     * @param _joint_ang_acc Angular acceleration, one value per joint.
     */
    void set_joint_ang_acc(const Eigen::VectorXd &_joint_ang_acc);
    /**
     * Sets joint angular acceleration from a standard vector and marks it available.
     * @param _joint_ang_acc Angular acceleration, one value per joint.
     */
    void set_joint_ang_acc(const std::vector<double> &_joint_ang_acc);
    /**
     * Sets joint torque and marks it available.
     * @param _joint_torque Torque, one value per joint.
     */
    void set_joint_torque(const Eigen::VectorXd &_joint_torque);
    /**
     * Sets joint torque from a standard vector and marks it available.
     * @param _joint_torque Torque, one value per joint.
     */
    void set_joint_torque(const std::vector<double> &_joint_torque);
    /**
     * Sets joint temperature and marks it available.
     * @param _joint_temp Temperature, one value per joint.
     */
    void set_joint_temp(const Eigen::VectorXd &_joint_temp);
    /**
     * Sets joint temperature from a standard vector and marks it available.
     * @param _joint_temp Temperature, one value per joint.
     */
    void set_joint_temp(const std::vector<double> &_joint_temp);
    /**
     * Sets joint power and marks it available.
     * @param _joint_power Power, one value per joint.
     */
    void set_joint_power(const Eigen::VectorXd &_joint_power);
    /**
     * Sets joint power from a standard vector and marks it available.
     * @param _joint_power Power, one value per joint.
     */
    void set_joint_power(const std::vector<double> &_joint_power);

    /**
     * Sets the measurement timestamp from a system-clock time value.
     * @param _timestamp Timestamp to store.
     */
    void set_time_stamp(const std::time_t &_timestamp);
    /**
     * Sets the measurement timestamp from a system-clock time point.
     * @param _timestamp Timestamp to store.
     */
    void set_time_stamp(const std::chrono::system_clock::time_point &_timestamp);

    /** @return A copy of the TCP pose. */
    CartesianSpace::Pose get_tcp_pose() const;
    /** @return A copy of the TCP velocity. */
    CartesianSpace::Pose get_tcp_vel() const;
    /** @return A copy of the TCP acceleration. */
    CartesianSpace::Pose get_tcp_acc() const;
    /** @return The six-axis TCP force/torque vector. */
    Eigen::Vector<double, 6> get_tcp_ft() const;
    /** @return Joint angles. */
    Eigen::VectorXd get_joint_angles() const;
    /** @return Joint angular velocity. */
    Eigen::VectorXd get_joint_ang_vel() const;
    /** @return Joint angular acceleration. */
    Eigen::VectorXd get_joint_ang_acc() const;
    /** @return Joint torque. */
    Eigen::VectorXd get_joint_torque() const;
    /** @return Joint temperature. */
    Eigen::VectorXd get_joint_temp() const;
    /** @return Joint power. */
    Eigen::VectorXd get_joint_power() const;
    /** @return Timestamp converted to std::time_t seconds. */
    std::time_t get_timestamp() const;
    /** @return Timestamp in milliseconds since the Unix epoch. */
    std::int64_t get_timestamp_ms() const;

    /**
     * @return True if joint_power is available.
     */
    bool get_is_available_joint_power() const
    {
        return is_available_joint_power;
    }

    /**
     * @return True if joint_temp is available.
     */
    bool get_is_available_joint_temp() const
    {
        return is_available_joint_temp;
    }

    /**
     * @return True if tcp_pose is available.
     */
    bool get_is_available_tcp_pose() const
    {
        return is_available_tcp_pose;
    }

    /**
     * @return True if tcp_vel is available.
     */
    bool get_is_available_tcp_vel() const
    {
        return is_available_tcp_vel;
    }

    /**
     * @return True if tcp_acc is available.
     */
    bool get_is_available_tcp_acc() const
    {
        return is_available_tcp_acc;
    }

    /**
     * @return True if tcp_ft is available.
     */
    bool get_is_available_tcp_ft() const
    {
        return is_available_tcp_ft;
    }

    /**
     * @return True if joint_angles is available.
     */
    bool get_is_available_joint_angles() const
    {
        return is_available_joint_angles;
    }

    /**
     * @return True if joint_ang_vel is available.
     */
    bool get_is_available_joint_ang_vel() const
    {
        return is_available_joint_ang_vel;
    }

    /**
     * @return True if joint_ang_acc is available.
     */
    bool get_is_available_joint_ang_acc() const
    {
        return is_available_joint_ang_acc;
    }

    /**
     * @return True if joint_torque is available.
     */
    bool get_is_available_joint_torque() const
    {
        return is_available_joint_torque;
    }

    /** @return True if a measurement timestamp is available. */
    bool get_is_available_timestamp() const
    {
        return is_available_timestamp;
    }

    /** Destroys the measurement. */
    ~Measurement();
};

#endif // MEASUREMENT_HPP