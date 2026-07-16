#ifndef TIMING_DIAGNOSTICS_H
#define TIMING_DIAGNOSTICS_H

#include <array>
#include <atomic>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

#include <ros/ros.h>

enum class PolicyWaitExitReason {
    SimPeriodReached,
    WallOvertime,
    Shutdown,
    SimTimeReset,
};

const char *policyWaitExitReasonName(PolicyWaitExitReason reason);

struct TimingRecord {
    const char *event = "UNKNOWN";
    std::uint64_t wall_time_ns = 0;
    std::uint64_t sim_time_us = 0;
    std::int64_t sim_dt_us = 0;
    double estimated_rtf = 0.0;
    std::uint64_t state_sequence = 0;
    std::uint64_t state_stamp_us = 0;
    std::uint64_t fsm_iteration = 0;
    bool control_update_accepted = false;
    std::uint64_t policy_sequence = 0;
    std::uint64_t policy_source_state_sequence = 0;
    std::uint64_t policy_sim_time_us = 0;
    std::uint64_t policy_wall_time_ns = 0;
    const char *policy_wait_exit_reason = "";
    std::int64_t policy_wait_sim_elapsed_us = 0;
    std::uint64_t policy_wait_wall_elapsed_us = 0;
    std::uint64_t history_oldest_stamp_us = 0;
    std::uint64_t history_newest_stamp_us = 0;
    std::uint64_t history_span_us = 0;
    std::uint64_t history_duplicate_count = 0;
    std::uint64_t action_sequence = 0;
    std::uint64_t action_source_state_sequence = 0;
    std::uint64_t lowcmd_sequence = 0;
    std::uint64_t lowcmd_action_sequence = 0;
    std::uint64_t lowcmd_sim_time_us = 0;
    bool torn_action = false;
};

class TimingDiagnostics {
public:
    static TimingDiagnostics &instance();

    void configure(const ros::NodeHandle &nh);
    bool enabled() const;
    void record(const TimingRecord &record);

    std::uint64_t beginActionWrite();
    std::uint64_t endActionWrite(std::uint64_t actionSequence,
                                 std::uint64_t sourceStateSequence,
                                 std::uint64_t sourceSimTimeUs);
    std::uint64_t actionWriteGeneration() const;
    std::uint64_t latestActionSequence() const;
    std::uint64_t latestActionSourceStateSequence() const;
    std::uint64_t latestActionSourceSimTimeUs() const;

private:
    TimingDiagnostics() = default;
    ~TimingDiagnostics();
    TimingDiagnostics(const TimingDiagnostics &) = delete;
    TimingDiagnostics &operator=(const TimingDiagnostics &) = delete;

    mutable std::mutex mutex_;
    std::ofstream stream_;
    std::atomic_bool configured_{false};
    std::atomic_bool enabled_{false};
    std::atomic<std::uint64_t> action_write_generation_{0};
    std::atomic<std::uint64_t> latest_action_sequence_{0};
    std::atomic<std::uint64_t> latest_action_source_state_sequence_{0};
    std::atomic<std::uint64_t> latest_action_source_sim_time_us_{0};
};

#endif  // TIMING_DIAGNOSTICS_H
