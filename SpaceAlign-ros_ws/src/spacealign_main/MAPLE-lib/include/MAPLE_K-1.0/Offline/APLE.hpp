#ifndef APLE_HPP
#define APLE_HPP

/** @file APLE.hpp
 *  @brief Offline recovery planning and action-sequence evaluation.
 */

#include <vector>
#include <memory>
#include <type_traits>
#include <functional>
#include <map>
#include <utility>

#include <eigen3/Eigen/Core>

#include "Offline/RecoveryStrategy.hpp"

namespace Offline::detail
{
    /** Internal recovery-planning engine used by the public APLE adapter. */
    class APLECore
    {
    private:
        struct Impl;
        std::unique_ptr<Impl> impl;

    public:
        /**
         * Creates a planner and loads its table from path when present.
         * Otherwise, the table is configured at path/S_A_R_T.xml.
         * @param path Directory used to persist the state-action-reward table.
         * @param simulate Callback that scores a candidate action sequence.
         * @param load_state Callback that restores simulator state from a path.
         */
        APLECore(const std::string &path, SimulationFunction simulate, LoadStateFunction load_state);

        /**
         * Copies planner state and installs a simulation callback for the copy.
         * @param other Planner state to copy.
         * @param simulate Callback used to score candidate action sequences.
         */
        APLECore(const APLECore &other, SimulationFunction simulate);

        /** Saves the state-action-reward table to its configured file. */
        ~APLECore();

        /**
         * Selects a recovery plan using the supplied error log and simulator state.
         * @param error_file Error-log path; a missing extension is treated as .xml.
         * @param environment_file Simulator-state path passed to the load callback.
         * @param relaxer Supplies the maximum constraint-relaxation displacement.
         * @return Candidate actions in execution order and any required relaxation.
         */
        Plan get_solution(std::string error_file, std::string environment_file, ConstraintRelaxer &relaxer);

        /**
         * Runs the callback within the optional constraint-relaxation scope.
         * @param execute Callback that performs the recovery actions.
         * @param relaxation Relaxation to apply, if the plan requires one.
         * @param relaxer Applies and clears the relaxation.
         * @return False if the scope cannot be established; otherwise the callback result.
         * @throws std::runtime_error If the relaxation cannot be cleared after execution.
         */
        bool execute_mitigation(std::function<bool()> execute, const std::optional<ConstraintRelaxation> &relaxation, ConstraintRelaxer &relaxer);

        /**
         * Writes the table as comma-separated rows.
         * @param filename Destination file path.
         * @throws std::runtime_error If the file cannot be opened for writing.
         */
        void save_state_action_reward_table(const std::string &filename);

        /**
         * Loads comma-separated rows into the table.
         * @param filename Source file path.
         * @throws std::runtime_error If the file cannot be opened or its dimensions differ from the table.
         * @throws std::invalid_argument If a table cell is not a valid number.
         * @throws std::out_of_range If a table cell is outside the representable numeric range.
         */
        void load_state_action_reward_table(const std::string &filename);

        /** @return A copy of the state-action-reward table. */
        Eigen::MatrixXd get_state_action_reward_table() const;
    };
}

namespace Offline
{
    /** Performance information produced while selecting a recovery solution. */
    struct RecoverPerformance
    {
        /** Aggregate criterion used to compare candidate solutions. */
        double criteria;
    };

    /**
     * @brief APLE offline recovery planner combining strategy selection and evaluation.
     * @tparam extAction External enum used by the host program's action API.
     */
    template <typename extAction>
    class APLE
    {
        static_assert(std::is_enum_v<extAction>, "APLE requires an enum action type.");

    private:
        using ActionSequence = std::vector<Recovery::Action<extAction>>;
        using SimulationFunction = std::function<double(const ActionSequence &)>;

        struct Interfaces
        {
            std::map<detail::ActionKind, extAction> action_space;
            SimulationFunction simulate;
        };

        std::shared_ptr<Interfaces> interfaces;
        std::unique_ptr<detail::APLECore> core;

        static ActionSequence to_external_actions(const detail::ActionSequence &actions, const Interfaces &interfaces)
        {
            ActionSequence converted;
            converted.reserve(actions.size());
            for (const auto &action : actions)
            {
                converted.push_back({action.target, interfaces.action_space.at(action.action)});
            }
            return converted;
        }

        static detail::SimulationFunction simulation_interface(std::shared_ptr<Interfaces> shared_interfaces)
        {
            return [owned_interfaces = std::move(shared_interfaces)](const detail::ActionSequence &actions)
            {
                return owned_interfaces->simulate(to_external_actions(actions, *owned_interfaces));
            };
        }

    public:
        /**
         * @brief Constructs APLE with simulation and state-loading callbacks.
         * @param path Directory where the state-action-reward table is stored.
         * @param _sim_func Callback that scores an action sequence.
         * @param load_sim_func Callback that restores simulator state by path.
         */
        APLE(const std::string &path, std::function<double(const std::vector<Recovery::Action<extAction>> &)> _sim_func, std::function<void(std::string)> load_sim_func)
            : interfaces(std::make_shared<Interfaces>(Interfaces{{}, std::move(_sim_func)})),
              core(std::make_unique<detail::APLECore>(path, simulation_interface(interfaces), std::move(load_sim_func)))
        {
        }

        /** Copies planner state and callback mappings into an independent instance. */
        APLE(const APLE &other)
            : interfaces(std::make_shared<Interfaces>(*other.interfaces)),
              core(std::make_unique<detail::APLECore>(*other.core, simulation_interface(interfaces)))
        {
        }

        /**
         * Replaces this APLE with an independent copy of other.
         * @param other APLE state and callback mappings to copy.
         * @return Reference to this planner.
         */
        APLE &operator=(const APLE &other)
        {
            if (this != &other)
            {
                APLE copy(other);
                interfaces.swap(copy.interfaces);
                core.swap(copy.core);
            }
            return *this;
        }

        /** Transfers planner state and callback mappings from another instance. */
        APLE(APLE &&) noexcept = default;

        /** Transfers planner state and callback mappings from another instance. */
        APLE &operator=(APLE &&) noexcept = default;

        /**
         * Registers external values for linear, joint, and motion-delta actions.
         * @param moveL External value used for linear motion.
         * @param moveJ External value used for joint motion.
         * @param motion_delta External value used for reduced-motion actions.
         */
        void set_action_space(extAction moveL, extAction moveJ, extAction motion_delta)
        {
            interfaces->action_space.insert_or_assign(detail::ActionKind::MOVE_L, moveL);
            interfaces->action_space.insert_or_assign(detail::ActionKind::MOVE_J, moveJ);
            interfaces->action_space.insert_or_assign(detail::ActionKind::MOTION_DELTA, motion_delta);
        }

        /**
         * @brief Finds the best known or newly evaluated recovery sequence.
         * @param failed_action External action that failed.
         * @param path_error_file XML file containing the failure context.
         * @param path_env_file Saved simulator state to load for evaluation.
         * @param relaxer Constraint relaxer used while testing candidate plans.
         * @return Recovery actions in execution order and any required relaxation.
         */
        Recovery::Plan<extAction> get_solution(extAction failed_action, std::string path_error_file, std::string path_env_file, ConstraintRelaxer &relaxer)
        {
            interfaces->action_space.insert_or_assign(detail::ActionKind::FAILED_ACTION, failed_action);
            const auto plan = core->get_solution(std::move(path_error_file), std::move(path_env_file), relaxer);
            return {to_external_actions(plan.actions, *interfaces), plan.relaxation};
        }

        /**
         * @brief Executes a recovery plan within its constraint-relaxation scope.
         * @param execute Callback that runs the plan's actions and reports whether execution succeeded.
         * @param recovery_plan Recovery plan, typically obtained from get_solution.
         * @param relaxer Relaxer used to apply and clear the plan's relaxation.
         * @return False if the relaxation scope cannot be established; otherwise the callback result.
         * @throws std::runtime_error If the relaxation scope cannot be cleared after execution.
         */
        bool execute_mitigation(std::function<bool(const std::vector<Recovery::Action<extAction>> &)> execute, const Recovery::Plan<extAction> &recovery_plan, ConstraintRelaxer &relaxer)
        {
            return core->execute_mitigation([&]
                                            { return execute(recovery_plan.actions); }, recovery_plan.relaxation, relaxer);
        }

        /**
         * @brief Writes the state-action-reward table to a CSV file.
         * @param filename Output file path.
         * @throws std::runtime_error If the file cannot be opened for writing.
         */
        void save_state_action_reward_table(const std::string &filename)
        {
            core->save_state_action_reward_table(filename);
        }

        /**
         * @brief Loads the state-action-reward table from a CSV file.
         * @param filename Input file path.
         * @throws std::runtime_error If the file cannot be read or its dimensions do not match.
         * @throws std::invalid_argument If a table cell is not a valid number.
         * @throws std::out_of_range If a table cell is outside the representable numeric range.
         */
        void load_state_action_reward_table(const std::string &filename)
        {
            core->load_state_action_reward_table(filename);
        }

        /** @return A copy of the state-action-reward table. */
        Eigen::MatrixXd get_state_action_reward_table() const
        {
            return core->get_state_action_reward_table();
        }

        /** Saves the state-action-reward table to its configured S_A_R_T.xml file. */
        ~APLE() = default;
    };
} // namespace Offline::Recovery

#endif // APLE_HPP
