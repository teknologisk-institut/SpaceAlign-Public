#ifndef MAPLE_ERROR_HPP
#define MAPLE_ERROR_HPP

/** @file Error.hpp
 *  @brief Analysis failure identifiers and their text labels.
 */
#include <string>
#include <map>

namespace Error
{
    /** Identifies an analysis failure or invalid measurement condition. */
    enum ReasonID
    {
        UNDEFINED = 0, ///< No specific analysis reason is available.
        LIM_NOT_SPECIFIED, ///< A required constraint limit was not configured.
        NO_FORCE, ///< Joint-space contact detection found no force.
        TORQUE_TOO_HIGH, ///< Joint torque exceeded its upper bound.
        TORQUE_TOO_LOW, ///< Joint torque fell below its lower bound.
        TORQUE_NORM_TOO_HIGH, ///< Joint torque norm exceeded its upper bound.
        TORQUE_NORM_TOO_LOW, ///< Joint torque norm fell below its lower bound.
        FORCE_TOO_EARLY, ///< Contact force was detected before the expected time.
        FORCE_TOO_LATE, ///< Contact force was detected after the expected time.
        TEMP_TOO_HIGH, ///< Joint temperature exceeded its upper bound.
        TEMP_TOO_LOW, ///< Joint temperature fell below its lower bound.
        POWER_TOO_HIGH, ///< Joint power exceeded its upper bound.
        POWER_TOO_LOW, ///< Joint power fell below its lower bound.
        JOINT_ANG_VEL_TOO_HIGH, ///< Joint angular velocity exceeded its upper bound.
        JOINT_ANG_VEL_TOO_LOW, ///< Joint angular velocity fell below its lower bound.
        JOINT_ANG_ACC_TOO_HIGH, ///< Joint angular acceleration exceeded its upper bound.
        JOINT_ANG_ACC_TOO_LOW, ///< Joint angular acceleration fell below its lower bound.
        FORCE_COMPONENT_TOO_LOW, ///< A TCP force component fell below its lower bound.
        FORCE_COMPONENT_TOO_HIGH, ///< A TCP force component exceeded its upper bound.
        FORCE_NORM_TOO_LOW, ///< TCP force norm fell below its lower bound.
        FORCE_NORM_TOO_HIGH, ///< TCP force norm exceeded its upper bound.
        TORQUE_COMPONENT_TOO_LOW, ///< A TCP torque component fell below its lower bound.
        TORQUE_COMPONENT_TOO_HIGH, ///< A TCP torque component exceeded its upper bound.
        TCP_TORQUE_NORM_TOO_LOW, ///< TCP torque norm fell below its lower bound.
        TCP_TORQUE_NORM_TOO_HIGH, ///< TCP torque norm exceeded its upper bound.
        TCP_VEL_COMPONENT_TOO_LOW, ///< A TCP velocity component fell below its lower bound.
        TCP_VEL_COMPONENT_TOO_HIGH, ///< A TCP velocity component exceeded its upper bound.
        TCP_VEL_NORM_TOO_LOW, ///< TCP velocity norm fell below its lower bound.
        TCP_VEL_NORM_TOO_HIGH, ///< TCP velocity norm exceeded its upper bound.
        TCP_ACC_COMPONENT_TOO_LOW, ///< A TCP acceleration component fell below its lower bound.
        TCP_ACC_COMPONENT_TOO_HIGH, ///< A TCP acceleration component exceeded its upper bound.
        TCP_ACC_NORM_TOO_LOW, ///< TCP acceleration norm fell below its lower bound.
        TCP_ACC_NORM_TOO_HIGH, ///< TCP acceleration norm exceeded its upper bound.
        TCP_ANG_VEL_TOO_LOW, ///< TCP angular velocity fell below its lower bound.
        TCP_ANG_VEL_TOO_HIGH, ///< TCP angular velocity exceeded its upper bound.
        TCP_ANG_ACC_TOO_LOW, ///< TCP angular acceleration fell below its lower bound.
        TCP_ANG_ACC_TOO_HIGH, ///< TCP angular acceleration exceeded its upper bound.
        ANAL_FW_TORQUE, ///< Joint torque analysis was enabled but torque data was unavailable.
        ANAL_FW_TEMP, ///< Temperature analysis was enabled but temperature data was unavailable.
        ANAL_FW_POWER, ///< Power analysis was enabled but power data was unavailable.
        ANAL_FW_ANG_ACC, ///< Angular-acceleration analysis was enabled but data was unavailable.
        ANAL_FW_ANG_VEL, ///< Angular-velocity analysis was enabled but data was unavailable.
        ANAL_IK_POSE, ///< Cartesian analysis required an unavailable TCP pose.
        ANAL_IK_FT, ///< Force/torque analysis required unavailable TCP force/torque data.
        ANAL_IK_VEL, ///< Velocity analysis required unavailable TCP velocity data.
        ANAL_IK_ACC, ///< Acceleration analysis required unavailable TCP acceleration data.
        ANAL_IK_ANG_VEL, ///< Angular-velocity analysis required unavailable TCP data.
        ANAL_IK_ANG_ACC, ///< Angular-acceleration analysis required unavailable TCP data.
        /** Sentinel marking the number of reason identifiers. */
        __LAST
    };

    /** Human-readable text associated with each error reason. */
    inline std::map<ReasonID, std::string> ReasonID_to_string{
        {UNDEFINED, "Undefined"},
        {LIM_NOT_SPECIFIED, "Limit not specified"},
        {NO_FORCE, "[Joint Space] No force detected"},
        {TORQUE_TOO_HIGH, "[Joint Space] Torque too high"},
        {TORQUE_TOO_LOW, "[Joint Space] Torque too low"},
        {TORQUE_NORM_TOO_HIGH, "[Joint Space] Norm of torque vector too high"},
        {TORQUE_NORM_TOO_LOW, "[Joint Space] Norm of torque vector too low"},
        {FORCE_TOO_EARLY, "[Joint Space] Force detected too early"},
        {FORCE_TOO_LATE, "[Joint Space] Force detected too late"},
        {TEMP_TOO_HIGH, "[Joint Space] Temperature too high"},
        {TEMP_TOO_LOW, "[Joint Space] Temperature too low"},
        {POWER_TOO_HIGH, "[Joint Space] Power too high"},
        {POWER_TOO_LOW, "[Joint Space] Power too low"},
        {JOINT_ANG_VEL_TOO_HIGH, "[Joint Space] Joint angle velocity is too high"},
        {JOINT_ANG_VEL_TOO_LOW, "[Joint Space] Joint angle velocity is too low"},
        {JOINT_ANG_ACC_TOO_HIGH, "[Joint Space] Joint angle acceleration is too high"},
        {JOINT_ANG_ACC_TOO_LOW, "[Joint Space] Joint angle acceleration is too low"},
        {FORCE_COMPONENT_TOO_LOW, "TCP force component was too low"},
        {FORCE_COMPONENT_TOO_HIGH, "TCP force component was too high"},
        {FORCE_NORM_TOO_LOW, "TCP force norm was too low"},
        {FORCE_NORM_TOO_HIGH, "TCP force norm was too high"},
        {TORQUE_COMPONENT_TOO_LOW, "TCP torque component was too low"},
        {TORQUE_COMPONENT_TOO_HIGH, "TCP torque component was too high"},
        {TCP_TORQUE_NORM_TOO_LOW, "TCP torque norm was too low"},
        {TCP_TORQUE_NORM_TOO_HIGH, "TCP torque norm was too high"},
        {TCP_VEL_COMPONENT_TOO_LOW, "TCP velocity component was too low"},
        {TCP_VEL_COMPONENT_TOO_HIGH, "TCP velocity component was too high"},
        {TCP_VEL_NORM_TOO_LOW, "TCP velocity norm was too low"},
        {TCP_VEL_NORM_TOO_HIGH, "TCP velocity norm was too high"},
        {TCP_ACC_COMPONENT_TOO_LOW, "TCP velocity component was too low"},
        {TCP_ACC_COMPONENT_TOO_HIGH, "TCP velocity component was too high"},
        {TCP_ACC_NORM_TOO_LOW, "TCP velocity norm was too low"},
        {TCP_ACC_NORM_TOO_HIGH, "TCP velocity norm was too high"},
        {TCP_ANG_VEL_TOO_LOW, "TCP angular velocity was too low"},
        {TCP_ANG_VEL_TOO_HIGH, "TCP angular velocity was too high"},
        {TCP_ANG_ACC_TOO_LOW, "TCP angular acceleration was too low"},
        {TCP_ANG_ACC_TOO_HIGH, "TCP angular acceleration was too high"},
        {ANAL_FW_TORQUE, "Measurement has no joint torque, but torque analysis is enabled"},
        {ANAL_FW_TEMP, "Measurement has no joint temperature, but temperature analysis is enabled"},
        {ANAL_FW_POWER, "Measurement has no joint power, but power analysis is enabled"},
        {ANAL_FW_ANG_ACC, "Measurement has no joint angular acceleration, but angular acceleration analysis is enabled"},
        {ANAL_FW_ANG_VEL, "Measurement has no joint angular velocity, but angular velocity analysis is enabled"},
        {ANAL_IK_POSE, "Measurement has no TCP pose, but Cartesian analysis needs pose for zone lookup"},
        {ANAL_IK_FT, "Measurement has no TCP force/torque, but force or torque analysis is enabled"},
        {ANAL_IK_VEL, "Measurement has no TCP velocity, but velocity analysis is enabled"},
        {ANAL_IK_ACC, "Measurement has no TCP acceleration, but acceleration analysis is enabled"},
        {ANAL_IK_ANG_VEL, "Measurement has no TCP velocity, but angular velocity analysis is enabled"},
        {ANAL_IK_ANG_ACC, "Measurement has no TCP acceleration, but angular acceleration analysis is enabled"}};

} // namespace Error

#endif