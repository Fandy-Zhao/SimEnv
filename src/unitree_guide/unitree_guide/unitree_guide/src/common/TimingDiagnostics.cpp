#include "common/TimingDiagnostics.h"

#include <iomanip>
#include <iostream>

const char *policyWaitExitReasonName(PolicyWaitExitReason reason) {
    switch (reason) {
    case PolicyWaitExitReason::SimPeriodReached:
        return "SIM_PERIOD_REACHED";
    case PolicyWaitExitReason::WallOvertime:
        return "WALL_OVERTIME";
    case PolicyWaitExitReason::Shutdown:
        return "SHUTDOWN";
    case PolicyWaitExitReason::SimTimeReset:
        return "SIM_TIME_RESET";
    }
    return "UNKNOWN";
}

TimingDiagnostics &TimingDiagnostics::instance() {
    static TimingDiagnostics diagnostics;
    return diagnostics;
}

TimingDiagnostics::~TimingDiagnostics() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stream_.is_open()) {
        stream_.flush();
        stream_.close();
    }
}

void TimingDiagnostics::configure(const ros::NodeHandle &nh) {
    bool expected = false;
    if (!configured_.compare_exchange_strong(expected, true)) {
        return;
    }

    bool requested = false;
    std::string path = "logs/unitree_timing.csv";
    nh.param("timing_diagnostics_enabled", requested, false);
    nh.param<std::string>("timing_diagnostics_path", path, path);
    if (!requested) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    stream_.open(path, std::ios::out | std::ios::trunc);
    if (!stream_.is_open()) {
        ROS_ERROR("Unable to open timing diagnostics CSV: %s", path.c_str());
        return;
    }
    stream_ << "event,wall_time_ns,sim_time_us,sim_dt_us,estimated_rtf,"
               "state_sequence,state_stamp_us,fsm_iteration,control_update_accepted,"
               "policy_sequence,policy_source_state_sequence,policy_sim_time_us,"
               "policy_wall_time_ns,policy_wait_exit_reason,policy_wait_sim_elapsed_us,"
               "policy_wait_wall_elapsed_us,history_oldest_stamp_us,"
               "history_newest_stamp_us,history_span_us,history_duplicate_count,"
               "action_sequence,action_source_state_sequence,lowcmd_sequence,"
               "lowcmd_action_sequence,lowcmd_sim_time_us,torn_action\n";
    enabled_.store(true, std::memory_order_release);
    ROS_INFO("Timing diagnostics enabled: %s", path.c_str());
}

bool TimingDiagnostics::enabled() const {
    return enabled_.load(std::memory_order_acquire);
}

void TimingDiagnostics::record(const TimingRecord &r) {
    if (!enabled()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    stream_ << r.event << ',' << r.wall_time_ns << ',' << r.sim_time_us << ','
            << r.sim_dt_us << ',' << std::setprecision(9) << r.estimated_rtf << ','
            << r.state_sequence << ',' << r.state_stamp_us << ',' << r.fsm_iteration << ','
            << (r.control_update_accepted ? 1 : 0) << ',' << r.policy_sequence << ','
            << r.policy_source_state_sequence << ',' << r.policy_sim_time_us << ','
            << r.policy_wall_time_ns << ',' << r.policy_wait_exit_reason << ','
            << r.policy_wait_sim_elapsed_us << ',' << r.policy_wait_wall_elapsed_us << ','
            << r.history_oldest_stamp_us << ',' << r.history_newest_stamp_us << ','
            << r.history_span_us << ',' << r.history_duplicate_count << ','
            << r.action_sequence << ',' << r.action_source_state_sequence << ','
            << r.lowcmd_sequence << ',' << r.lowcmd_action_sequence << ','
            << r.lowcmd_sim_time_us << ',' << (r.torn_action ? 1 : 0) << '\n';
}

std::uint64_t TimingDiagnostics::beginActionWrite() {
    return action_write_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
}

std::uint64_t TimingDiagnostics::endActionWrite(std::uint64_t sourceStateSequence,
                                                std::uint64_t sourceSimTimeUs) {
    latest_action_source_state_sequence_.store(sourceStateSequence, std::memory_order_release);
    latest_action_source_sim_time_us_.store(sourceSimTimeUs, std::memory_order_release);
    const std::uint64_t sequence = latest_action_sequence_.fetch_add(1, std::memory_order_acq_rel) + 1;
    action_write_generation_.fetch_add(1, std::memory_order_release);
    return sequence;
}

std::uint64_t TimingDiagnostics::actionWriteGeneration() const {
    return action_write_generation_.load(std::memory_order_acquire);
}

std::uint64_t TimingDiagnostics::latestActionSequence() const {
    return latest_action_sequence_.load(std::memory_order_acquire);
}

std::uint64_t TimingDiagnostics::latestActionSourceStateSequence() const {
    return latest_action_source_state_sequence_.load(std::memory_order_acquire);
}

std::uint64_t TimingDiagnostics::latestActionSourceSimTimeUs() const {
    return latest_action_source_sim_time_us_.load(std::memory_order_acquire);
}
