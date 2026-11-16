#ifndef CONSTRAINTRELAXATION_HPP
#define CONSTRAINTRELAXATION_HPP

/** @file ConstraintRelaxation.hpp
 *  @brief Constraint relaxation requests and their execution scope.
 */
#include <string>
#include <optional>

#include "Error.hpp"

namespace CartesianSpace::Zones
{
    /** Identifies the robot's spatial relationship to the target. */
    enum State
    {
        /** Outside all task-specific regions. */
        FREE,
        /** Approaching or moving toward the target. */
        EXPECTED,
        /** In the target contact region. */
        IN_CONTACT,
        /** Moving away after contact. */
        PAST_CONTACT,
        /** No zone could be identified. */
        UNDEFINED
    };
}

/** Describes one constraint-bound adjustment for a recovery attempt. */
struct ConstraintRelaxation
{
    /** Error reason identifying the violated boundary to widen. */
Error::ReasonID reason = Error::ReasonID::UNDEFINED;
    /** Cartesian zone to adjust; UNDEFINED can represent all applicable zones. */
    CartesianSpace::Zones::State zone = CartesianSpace::Zones::UNDEFINED;
    /** Amount by which the selected bound is widened, in that limit's units. */
    double displacement = 0.0;

    /** Resets the reason, zone, and displacement to their undefined defaults. */
    void reset()
    {
        reason = Error::ReasonID::UNDEFINED;
        zone = CartesianSpace::Zones::UNDEFINED;
        displacement = 0.0;
    };
};

/** Identifies which side of a limit is selected for relaxation. */
enum class Bound
{
    /** Upper limit. */
    MAX,
    /** Lower limit. */
    MIN,
    /** No bound is associated with the reason. */
    NONE
};

/**
 * Identifies which side of a configured limit is associated with an error.
 * @param _id Error reason whose affected bound should be identified.
 * @return The upper, lower, or no bound indicated by the reason.
 */
inline Bound bound_of(Error::ReasonID _id)
{

    if (_id == Error::__LAST)
    {
        return Bound::NONE;
    }

    std::string error_text = Error::ReasonID_to_string.at(_id);

    if (error_text.find("high") != std::string::npos)
    {
        return Bound::MAX;
    }
    else if (error_text.find("low") != std::string::npos)
    {
        return Bound::MIN;
    }
    else
    {
        return Bound::NONE;
    }
};

/**
 * Determines whether an error reason can be handled by widening a constraint.
 * @param _id Error reason to check.
 * @return True if the reason is relaxable.
 */
inline bool is_relaxable(Error::ReasonID _id)
{
    switch (_id)
    {
    case Error::UNDEFINED:
        return false;
    case Error::LIM_NOT_SPECIFIED:
        return false;
    case Error::NO_FORCE:
        return false;
    case Error::TORQUE_TOO_HIGH:
        return true;
    case Error::TORQUE_TOO_LOW:
        return true;
    case Error::TORQUE_NORM_TOO_HIGH:
        return true;
    case Error::TORQUE_NORM_TOO_LOW:
        return true;
    case Error::FORCE_TOO_EARLY:
        return false;
    case Error::FORCE_TOO_LATE:
        return false;
    case Error::TEMP_TOO_HIGH:
        return true;
    case Error::TEMP_TOO_LOW:
        return true;
    case Error::POWER_TOO_HIGH:
        return true;
    case Error::POWER_TOO_LOW:
        return true;
    case Error::JOINT_ANG_VEL_TOO_HIGH:
        return true;
    case Error::JOINT_ANG_VEL_TOO_LOW:
        return true;
    case Error::JOINT_ANG_ACC_TOO_HIGH:
        return true;
    case Error::JOINT_ANG_ACC_TOO_LOW:
        return true;
    case Error::FORCE_COMPONENT_TOO_LOW:
        return true;
    case Error::FORCE_COMPONENT_TOO_HIGH:
        return true;
    case Error::FORCE_NORM_TOO_LOW:
        return true;
    case Error::FORCE_NORM_TOO_HIGH:
        return true;
    case Error::TORQUE_COMPONENT_TOO_LOW:
        return true;
    case Error::TORQUE_COMPONENT_TOO_HIGH:
        return true;
    case Error::TCP_TORQUE_NORM_TOO_LOW:
        return true;
    case Error::TCP_TORQUE_NORM_TOO_HIGH:
        return true;
    case Error::TCP_VEL_COMPONENT_TOO_LOW:
        return true;
    case Error::TCP_VEL_COMPONENT_TOO_HIGH:
        return true;
    case Error::TCP_VEL_NORM_TOO_LOW:
        return true;
    case Error::TCP_VEL_NORM_TOO_HIGH:
        return true;
    case Error::TCP_ACC_COMPONENT_TOO_LOW:
        return true;
    case Error::TCP_ACC_COMPONENT_TOO_HIGH:
        return true;
    case Error::TCP_ACC_NORM_TOO_LOW:
        return true;
    case Error::TCP_ACC_NORM_TOO_HIGH:
        return true;
    case Error::TCP_ANG_VEL_TOO_LOW:
        return true;
    case Error::TCP_ANG_VEL_TOO_HIGH:
        return true;
    case Error::TCP_ANG_ACC_TOO_LOW:
        return true;
    case Error::TCP_ANG_ACC_TOO_HIGH:
        return true;
    case Error::ANAL_FW_TORQUE:
        return false;
    case Error::ANAL_FW_TEMP:
        return false;
    case Error::ANAL_FW_POWER:
        return false;
    case Error::ANAL_FW_ANG_ACC:
        return false;
    case Error::ANAL_FW_ANG_VEL:
        return false;
    case Error::ANAL_IK_POSE:
        return false;
    case Error::ANAL_IK_FT:
        return false;
    case Error::ANAL_IK_VEL:
        return false;
    case Error::ANAL_IK_ACC:
        return false;
    case Error::ANAL_IK_ANG_VEL:
        return false;
    case Error::ANAL_IK_ANG_ACC:
        return false;
    case Error::__LAST:
        return false;
    default:
        return false;
    }
};

/** Interface for applying and clearing a temporary constraint relaxation. */
struct ConstraintRelaxer
{
    /**
     * Applies the requested temporary constraint relaxation.
     * @param relaxation Relaxation request to apply.
     * @return True if the request was applied.
     */
    virtual bool apply_relaxation(const ConstraintRelaxation &relaxation) = 0;

    /** Clears the currently active relaxation and restores baseline constraints. */
    virtual void clear_relaxation() = 0;

    /** @return The configured maximum displacement, or empty if no maximum is set. */
    virtual std::optional<double> get_max_relax_displacement() const
    {
        return std::nullopt;
    }

    /** Destroys the relaxation interface. */
    virtual ~ConstraintRelaxer() = default;
};

/** Applies an optional relaxation for a scope and clears it on scope exit. */
class RelaxationScope
{
private:
    ConstraintRelaxer &relaxer_;
    bool applied_ = false;
    bool apply_succeeded_ = true;
    bool clear_attempted_ = false;
    bool clear_succeeded_ = true;

public:
    /**
     * Begins a relaxation scope and applies the request when one is supplied.
     * @param relaxer Interface that applies and clears the relaxation.
     * @param relaxation Optional request to apply.
     */
    RelaxationScope(ConstraintRelaxer &relaxer, const std::optional<ConstraintRelaxation> &relaxation)
        : relaxer_(relaxer)
    {
        if (relaxation)
        {
            apply_succeeded_ = relaxer_.apply_relaxation(*relaxation);
            applied_ = apply_succeeded_;
        }
    }

    /** Attempts to clear an applied relaxation when the scope ends. */
    ~RelaxationScope() noexcept
    {
        clear();
    }

    /** @return True if applying and clearing the relaxation have succeeded. */
    bool ok() const noexcept
    {
        return apply_succeeded_ && clear_succeeded_;
    }

    /**
     * Clears the relaxation at most once.
     * @return True if clearing succeeded or no clear was required.
     */
    bool clear() noexcept
    {
        if (!applied_ || clear_attempted_)
            return clear_succeeded_;

        clear_attempted_ = true;
        try
        {
            relaxer_.clear_relaxation();
        }
        catch (...)
        {
            clear_succeeded_ = false;
        }
        return clear_succeeded_;
    }

    /** Copying a relaxation scope is disabled to preserve single-owner cleanup. */
    RelaxationScope(const RelaxationScope &) = delete;
    /** Copy assignment is disabled to preserve single-owner cleanup. */
    RelaxationScope &operator=(const RelaxationScope &) = delete;
};

#endif // CONSTRAINTRELAXATION_HPP