#ifndef MAL_HPP
#define MAL_HPP

/** @file MAL.hpp
 *  @brief Real-time measurement analysis loop and host integration API.
 */

#include <cstdint>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include <atomic>
#include <functional>
#include <optional>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>

#include "Measurement.hpp"
#include "Analyse_Flags.hpp"
#include "ConstraintRelaxation.hpp"
using namespace std;

struct AnalyseConclusion;
class Analyse;
class Legitimate;

/** Runs real-time constraint analysis and coordinates stop, logging, and host callbacks. */
class MAL : public ConstraintRelaxer
{
private:
    // default setup
    string MAL_name;
    // stop Functionality
    function<void()> stop_signal;
    // Signalled once the automatic error log is written, flushed and closed.
    function<void()> log_saved_signal;
    // Knowledge base
    function<void()> update_environmental_knowledge;
    bool is_knowledge_updated = false;
    string env_knowledge_file = "";

    // Configuration variables
    bool is_configured = false;
    // Measurements
    std::shared_ptr<const Measurement> current_measurement;
    atomic<bool> measurement_received = false;
    // Boundaries
    std::unique_ptr<Legitimate> legitimater;
    string legitimate_conf_file = "";
    std::shared_ptr<Legitimate> active_legitimater;
    std::optional<ConstraintRelaxation> current_relaxation;
    // Analyse settings
    std::unique_ptr<Analyse> analyser;
    bool free_zone_restriction = false;
    double sample_freq = -1;
    atomic<bool> is_running_MAL_loop = false;
    boost::chrono::duration<double> period;
    int overrun_tolerance = 3;
    int consecutive_overruns = 0;

    // parallel
    unique_ptr<boost::thread> main_thread_ptr = nullptr;

    // functions
    /** Stops the system, writes the failure log, and notifies the host if the write succeeds. */
    void stopping_process(AnalyseConclusion &conc);

protected:
    /** Runs the periodic measurement-analysis loop until stopped or an error occurs. */
    void MAL_loop();

    /**
     * Appends an XML entry to filename + ".xml".
     * @param filename Output path without the automatically appended .xml suffix.
     * @param pt Property tree containing the XML entry to append.
     * @return True only if the stream was opened, written, and closed without error.
     */
    bool write_file(string filename, boost::property_tree::ptree pt);

    /**
     * Saves an automatic failure-log entry for the supplied analysis conclusion.
     * @param conc Analysis result that caused the failure handling.
     * @return True only if the entry was fully written and the stream closed.
     */
    bool save_log(const AnalyseConclusion &conc);

public:
    /**
     * Constructs the online checker without configuring its constraints or callbacks.
     * Call configure() before enable().
     * @param _name Unique identifier, such as the host application's state name.
     * @param _sample_freq Measurement sampling frequency in hertz.
     * @param log_time Duration in seconds retained in measurement history buffers.
     */
    MAL(string _name, double _sample_freq, double log_time);

    /**
     * Constructs and configures the online checker with its callbacks and constraints.
     * @param _name Unique identifier, such as the host application's state name.
     * @param stop_trigger Callback used to stop system movement after a failure.
     * @param update_knowledge Callback used to save or update environmental state.
     * @param config_file Path to the configuration file containing constraints and movement zones.
     * @param log_saved_trigger Optional callback notified when an automatic failure log is complete.
     * @param _sample_freq Measurement sampling frequency in hertz.
     * @param log_time Duration in seconds retained in measurement history buffers.
     */
    MAL(string _name, function<void()> stop_trigger, function<void()> update_knowledge, string config_file, function<void()> log_saved_trigger, double _sample_freq, double log_time);

    /**
     * Configures the online checker, constraints, callbacks, and active analysis flags.
     * Configuration is rejected while the analysis loop is running or a relaxation is active.
     * @param stop_trigger Callback used to stop system movement after a failure.
     * @param update_knowledge Callback used to save or update environmental state.
     * @param config_file Path to the configuration file containing constraints and movement zones.
     * @param log_saved_trigger Optional notification callback; empty preserves the registered callback.
     * @param cs_flag Cartesian-space measurements to analyse.
     * @param js_flag Joint-space measurements to analyse.
     */
    void configure(function<void()> stop_trigger, function<void()> update_knowledge, string config_file, function<void()> log_saved_trigger, CartesianSpace::AnalyseFlag cs_flag = CartesianSpace::AnalyseFlag::None, JointSpace::AnalyseFlag js_flag = JointSpace::AnalyseFlag::None);

    /**
     * Starts the analysis loop when MAL has been configured and is not already running.
     * @pre configure() has completed successfully.
     * @return True if the loop was started; false if it could not be started.
     */
    bool enable();

    /**
     * Requests that the analysis loop stop and waits up to 1.5 seconds for its thread to finish.
     * @return True if the thread was joined within the wait; false if no joinable thread
     *         was available or it did not finish in time.
     */
    bool disable();

    /**
     * Replaces the callback invoked to stop system movement after a detected failure.
     * @param stop_function Callback that triggers the system stop.
     */
    void set_stop_Trigger(function<void()> stop_function);

    /**
     * Function handle invoked when an automatic failure log has been completed.
     * It is called synchronously from the MAL loop thread, after stop_signal has run and
     * after the error-log XML entry has been written, flushed and closed, so the file is
     * complete and safe for the host to read (e.g. via Offline::read_latest_err_log).
     * It is not called for user-requested save(), or if writing the log failed.
     * Passing an empty callback clears the registration.
     * @param callback Function notified that the automatic failure log is ready.
     */
    void set_log_saved_trigger(function<void()> callback);

    /**
     * Publishes the latest measurement for analysis by the running loop.
     * @param m Shared pointer to the measurement; ownership is shared with MAL.
     */
    void update_measurement(std::shared_ptr<const Measurement> m);

    /**
     * Sets the joint-space target and its motion parameters for subsequent analysis.
     * @param target Target joint angles in radians.
     * @param vel Target velocity passed to the joint-space analyser.
     * @param acc Target acceleration passed to the joint-space analyser.
     */
    void set_js_target(Eigen::VectorXd target, double vel, double acc);

    /**
     * Sets the joint-space target and its motion parameters for subsequent analysis.
     * @param target Target joint angles in radians.
     * @param vel Target velocity passed to the joint-space analyser.
     * @param acc Target acceleration passed to the joint-space analyser.
     */
    void set_js_target(std::vector<double> target, double vel, double acc);

    /**
     * Selects which Cartesian- and joint-space measurements the analyser checks.
     * @param cs_flags Cartesian flags: None, Force, Torque, Velocity, Acceleration,
     *                 AngularVelocity, or AngularAcceleration.
     * @param js_flags Joint flags: None, Torque, Temperature, AngularVelocity,
     *                 AngularAcceleration, or Power.
     */
    void set_analyse_flags(CartesianSpace::AnalyseFlag cs_flags = CartesianSpace::AnalyseFlag::None, JointSpace::AnalyseFlag js_flags = JointSpace::AnalyseFlag::None);

    /**
     * Sets the Cartesian-space target and its motion parameters for subsequent analysis.
     * Constraints are selected according to the current zone.
     * @param target Target Cartesian pose.
     * @param vel Target velocity passed to the Cartesian-space analyser.
     * @param acc Target acceleration passed to the Cartesian-space analyser.
     */
    void set_cs_target(CartesianSpace::Pose target, double vel, double acc);

    /**
     * Restricts Cartesian-space constraint checks to the free zone.
     * @param enable True to enable free-zone-only checks; false to check the current zone normally.
     */
    void check_freeZone_constraints_only(bool enable);

    /**
     * Number of consecutive missed loop deadlines tolerated before the system is stopped.
     * @param tolerance 0 stops on the first missed deadline.
     * @throws std::invalid_argument If tolerance is negative.
     */
    void set_overrun_tolerance(int tolerance);

    /**
     * Appends a user-requested snapshot of measurements, target, and configured constraints.
     * The XML file is written to filename + ".xml".
     * @param filename File path without the automatically appended .xml suffix.
     */
    void save(string filename);

    /** Invokes the configured environmental-knowledge update callback and clears its updated state. */
    void update_env_knowledge();

    /**
     * Stores the path to the environmental-knowledge file.
     * A non-empty path marks the knowledge base as updated.
     * @param file_name Environmental-knowledge file path.
     */
    void set_env_knowledge_file(const string &file_name);

    /** @return The currently stored environmental-knowledge file path. */
    string get_env_knowledge_file();

    /** @return The identifier supplied when MAL was constructed. */
    string get_name();

    /**
     * Checks whether an environmental-knowledge file has been marked as updated.
     * The flag is cleared when knowledge is updated, MAL is configured, or a failure is handled.
     * @return True if the knowledge-updated flag is set.
     */
    bool is_knowledgebase_updated();

    /**
     * Applies a relaxation to the configured constraints while the analysis loop is stopped.
     * @param relaxation Constraint boundary and displacement to apply.
     * @return True if the relaxation was accepted and applied; false if MAL is not
     *         configured, is running, or the relaxation is invalid.
     */
    bool apply_relaxation(const ConstraintRelaxation &relaxation) override;

    /**
     * Clears the active relaxation and restores the configured baseline constraints.
     * Does nothing if no relaxation is active or MAL is not configured.
     * @throws std::logic_error If called while the analysis loop is running.
     */
    void clear_relaxation() override;

    /** @return The active relaxation, or std::nullopt when none is applied. */
    std::optional<ConstraintRelaxation> get_active_relaxation() const;

    /** Returns the configured maximum relaxation displacement, or empty when relaxation is disabled. */
    std::optional<double> get_max_relax_displacement() const override;

    /** Stops the analysis loop and joins its thread before destroying MAL. */
    ~MAL();
};

#endif // MAL_HPP