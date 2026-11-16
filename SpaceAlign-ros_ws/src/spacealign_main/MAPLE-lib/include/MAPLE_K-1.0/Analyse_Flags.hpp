#ifndef ANALYSE_FLAGS_HPP
#define ANALYSE_FLAGS_HPP

/** @file Analyse_Flags.hpp
 *  @brief Selects measurements for joint-space and Cartesian-space analysis.
 */

#include <cstdint>
namespace JointSpace
{
    /** Measurements that the joint-space analyser can check. */
    enum class AnalyseFlag : uint32_t
    {
        None = 0,               ///< Disable all joint-space checks.
        Torque = 1u << 0,       ///< Check joint torque.
        Temperature = 1u << 1,  ///< Check joint temperature.
        AngularVelocity = 1u << 2, ///< Check joint angular velocity.
        AngularAcceleration = 1u << 3, ///< Check joint angular acceleration.
        Power = 1u << 4,        ///< Check joint power.
    };

    /**
     * Combines two joint-space analysis flags.
     * @param lhs First flag set.
     * @param rhs Second flag set.
     * @return The bitwise union of lhs and rhs.
     */
    inline constexpr AnalyseFlag operator|(AnalyseFlag lhs, AnalyseFlag rhs)
    {
        return static_cast<AnalyseFlag>(static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs));
    }

    /**
     * Returns the joint-space flags present in both operands.
     * @param lhs First flag set.
     * @param rhs Second flag set.
     * @return The bitwise intersection of lhs and rhs.
     */
    inline constexpr AnalyseFlag operator&(AnalyseFlag lhs, AnalyseFlag rhs)
    {
        return static_cast<AnalyseFlag>(static_cast<uint32_t>(lhs) & static_cast<uint32_t>(rhs));
    }

    /**
     * Tests whether any bit in flag is set in flags.
     * @param flags Set of joint-space checks to inspect.
     * @param flag Flag bits to test.
     * @return True if the operands share at least one set bit.
     */
    inline constexpr bool has_flag(AnalyseFlag flags, AnalyseFlag flag)
    {
        return static_cast<uint32_t>(flags & flag) != 0u;
    }
}

namespace CartesianSpace
{
    /** Measurements that the Cartesian-space analyser can check. */
    enum class AnalyseFlag : uint32_t
    {
        None = 0,               ///< Disable all Cartesian-space checks.
        Force = 1u << 0,        ///< Check TCP force.
        Torque = 1u << 1,       ///< Check TCP torque.
        Velocity = 1u << 2,     ///< Check TCP linear velocity.
        Acceleration = 1u << 3, ///< Check TCP linear acceleration.
        AngularVelocity = 1u << 4, ///< Check TCP angular velocity.
        AngularAcceleration = 1u << 5, ///< Check TCP angular acceleration.
    };

    /**
     * Combines two Cartesian-space analysis flags.
     * @param lhs First flag set.
     * @param rhs Second flag set.
     * @return The bitwise union of lhs and rhs.
     */
    inline constexpr AnalyseFlag operator|(AnalyseFlag lhs, AnalyseFlag rhs)
    {
        return static_cast<AnalyseFlag>(static_cast<uint32_t>(lhs) | static_cast<uint32_t>(rhs));
    }

    /**
     * Returns the Cartesian-space flags present in both operands.
     * @param lhs First flag set.
     * @param rhs Second flag set.
     * @return The bitwise intersection of lhs and rhs.
     */
    inline constexpr AnalyseFlag operator&(AnalyseFlag lhs, AnalyseFlag rhs)
    {
        return static_cast<AnalyseFlag>(static_cast<uint32_t>(lhs) & static_cast<uint32_t>(rhs));
    }

    /**
     * Tests whether any bit in flag is set in flags.
     * @param flags Set of Cartesian-space checks to inspect.
     * @param flag Flag bits to test.
     * @return True if the operands share at least one set bit.
     */
    inline constexpr bool has_flag(AnalyseFlag flags, AnalyseFlag flag)
    {
        return static_cast<uint32_t>(flags & flag) != 0u;
    }
}
#endif // ANALYSE_FLAGS_HPP