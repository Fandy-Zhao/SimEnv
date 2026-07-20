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
               "state_sequence,state_stamp_us,fsm_iteration,fsm_sequence,"
               "estimator_sequence,estimator_source_state_sequence,wave_sequence,"
               "gait_cycle_sequence,reset_generation,runtime_ctrl_dt_us,"
               "target_control_hz,control_update_accepted,"
               "new_state_consumed,repeated_state_consumed,scheduler_accepted,"
               "accepted_state_sequence,scheduler_lag_us,missed_periods,"
               "repeated_state_rejected_count,fsm_state,wave_status,resolved_vx,"
               "resolved_vy,resolved_yaw_rate,"
               "phase_0,phase_1,phase_2,phase_3,contact_0,contact_1,contact_2,contact_3,"
               "policy_sequence,policy_source_state_sequence,policy_sim_time_us,"
               "policy_wall_time_ns,policy_wait_exit_reason,policy_wait_sim_elapsed_us,"
               "policy_wait_wall_elapsed_us,history_oldest_stamp_us,"
               "history_newest_stamp_us,history_span_us,history_duplicate_count,"
               "action_sequence,action_source_state_sequence,lowcmd_sequence,"
               "lowcmd_action_sequence,lowcmd_sim_time_us,torn_action," "prewave_readiness_flags,prewave_readiness_hold_elapsed," "prewave_first_block_reason,prewave_model_height," "prewave_numerical_guard_stage,prewave_wave_cancel_reason\n";
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
            << r.fsm_sequence << ',' << r.estimator_sequence << ','
            << r.estimator_source_state_sequence << ',' << r.wave_sequence << ','
            << r.gait_cycle_sequence << ',' << r.reset_generation << ','
            << r.runtime_ctrl_dt_us << ',' << r.target_control_hz << ','
            << (r.control_update_accepted ? 1 : 0) << ','
            << (r.new_state_consumed ? 1 : 0) << ','
            << (r.repeated_state_consumed ? 1 : 0) << ','
            << (r.scheduler_accepted ? 1 : 0) << ','
            << r.accepted_state_sequence << ',' << r.scheduler_lag_us << ','
            << r.missed_periods << ',' << r.repeated_state_rejected_count << ','
            << r.fsm_state << ',' << r.wave_status << ',' << r.resolved_vx << ','
            << r.resolved_vy << ',' << r.resolved_yaw_rate << ','
            << r.phase[0] << ',' << r.phase[1] << ',' << r.phase[2] << ','
            << r.phase[3] << ',' << r.contact[0] << ',' << r.contact[1] << ','
            << r.contact[2] << ',' << r.contact[3] << ','
            << r.policy_sequence << ','
            << r.policy_source_state_sequence << ',' << r.policy_sim_time_us << ','
            << r.policy_wall_time_ns << ',' << r.policy_wait_exit_reason << ','
            << r.policy_wait_sim_elapsed_us << ',' << r.policy_wait_wall_elapsed_us << ','
            << r.history_oldest_stamp_us << ',' << r.history_newest_stamp_us << ','
            << r.history_span_us << ',' << r.history_duplicate_count << ','
            << r.action_sequence << ',' << r.action_source_state_sequence << ','
            << r.lowcmd_sequence << ',' << r.lowcmd_action_sequence << ','
            << r.lowcmd_sim_time_us << ',' << (r.torn_action ? 1 : 0) << ',' << r.prewave_readiness_flags << ',' << std::setprecision(6) << r.prewave_readiness_hold_elapsed << ',' << r.prewave_first_block_reason << ',' << r.prewave_model_height << ',' << r.prewave_numerical_guard_stage << ',' << r.prewave_wave_cancel_reason << '\n';
}

std::uint64_t TimingDiagnostics::beginActionWrite() {
    return action_write_generation_.fetch_add(1, std::memory_order_acq_rel) + 1;
}

std::uint64_t TimingDiagnostics::endActionWrite(std::uint64_t actionSequence,
                                                std::uint64_t sourceStateSequence,
                                                std::uint64_t sourceSimTimeUs) {
    latest_action_source_state_sequence_.store(sourceStateSequence, std::memory_order_release);
    latest_action_source_sim_time_us_.store(sourceSimTimeUs, std::memory_order_release);
    latest_action_sequence_.store(actionSequence, std::memory_order_release);
    action_write_generation_.fetch_add(1, std::memory_order_release);
    return actionSequence;
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
