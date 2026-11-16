#ifndef OFFLINE_RECOVERY_STRATEGY_HPP
#define OFFLINE_RECOVERY_STRATEGY_HPP

#include <string>
#include <map>
#include <vector>
#include <optional>
#include <type_traits>
#include <functional>

#include "ConstraintRelaxation.hpp"

/** @file RecoveryStrategy.hpp
 *  @brief Recovery strategy identifiers and external recovery actions.
 */

namespace Offline::Recovery
{

    /** @brief Strategies that can be composed to recover from a safety error. */
    enum Strategy
    {
        CALL_FOR_HELP = 0, ///< Enable manual jogging or request human assistance.
        BACK_TRACE,        ///< Return to a previously recorded position.
        RETRY,             ///< Retry the original target from the current position.
        INCREASE_LIMIT,    ///< Increase the violated upper constraint bound.
        DECREASE_LIMIT,    ///< Decrease the violated lower constraint bound.
        RELAX,             ///< Move in the measured force or torque direction.
        REDUCE_VEL_ACC,    ///< Reduce velocity and acceleration to lower dynamic loads.
    };

    /** Human-readable descriptions for each recovery strategy. */
    static const std::map<Strategy, std::string> recoveryStrategy_to_string = {
        {Strategy::CALL_FOR_HELP, "Call for help: Enable manual jog or await human instruction"},
        {Strategy::BACK_TRACE, "Back trace: Move to a previously visited position"},
        {Strategy::RETRY, "Retry: Attempt the same move from current position"},
        {Strategy::INCREASE_LIMIT, "Increase limit: Loosen violated constraint higher bound"},
        {Strategy::DECREASE_LIMIT, "Decrease limit: Loosen violated constraint lower bound"},
        {Strategy::RELAX, "Relax: Move in the TCP force direction or Joint torque direction"},
        {Strategy::REDUCE_VEL_ACC, "Reduce velocity/acceleration: Lower dynamic load"},
    };

    /**
     * @brief An action understood by the external robot or simulator.
     * @tparam extActionT User-defined enum identifying an external action.
     */
    template <typename extActionT>
    struct Action
    {
        static_assert(std::is_enum_v<extActionT>, "Recovery action requires an enum Action type");
        /** Position or motion parameters passed to the external action. */
        std::vector<double> target;
        /** External action identifier. */
        extActionT action;
    };

    /**
     * @brief A complete recovery plan, including actions and optional relaxation.
     * @tparam extActionT User-defined enum identifying external actions.
     */
    template <typename extActionT>
    struct Plan
    {
        static_assert(std::is_enum_v<extActionT>, "Recovery Plan requires an enum plan type");
        /** Recovery actions in the order they should be executed. */
        std::vector<Action<extActionT>> actions;
        /** Optional constraint relaxation to apply while executing the plan. */
        std::optional<ConstraintRelaxation> relaxation;
    };
    
}// Offline::Recovery

namespace Offline::detail
{
    /** Internal action identifiers mapped to host-defined action values by APLE. */
    enum class ActionKind
    {
        MOVE_L,       ///< Linear/cartesian movement.
        MOVE_J,       ///< Joint-space movement.
        MOTION_DELTA, ///< Reduced-motion parameter update.
        FAILED_ACTION ///< Retry of the originally failed action.
    };

    /** Internal recovery action type used by the planner implementation. */
    using Action = Recovery::Action<ActionKind>;
    /** Internal recovery plan type used by the planner implementation. */
    using Plan = Recovery::Plan<ActionKind>;
    /** Ordered sequence of internal recovery actions. */
    using ActionSequence = std::vector<Action>;
    /** Callback that scores an internal recovery-action sequence. */
    using SimulationFunction = std::function<double(const ActionSequence &)>;
    /** Callback that restores simulator state from a path. */
    using LoadStateFunction = std::function<void(std::string)>;
}

#endif // OFFLINE_RECOVERY_STRATEGY_HPP
