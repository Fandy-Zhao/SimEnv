#ifndef TIMING_ALIGNMENT_H
#define TIMING_ALIGNMENT_H

#include <cstdint>
#include <limits>

#include "common/TimingDiagnostics.h"

inline PolicyWaitExitReason classifyPolicyWaitExit(std::uint64_t nowUs,
                                                    std::uint64_t startUs,
                                                    std::uint64_t periodUs,
                                                    bool shutdown){
    if(shutdown){
        return PolicyWaitExitReason::Shutdown;
    }
    if(nowUs < startUs){
        return PolicyWaitExitReason::SimTimeReset;
    }
    if(nowUs - startUs >= periodUs){
        return PolicyWaitExitReason::SimPeriodReached;
    }
    return PolicyWaitExitReason::WallOvertime;
}

class PolicyHistoryGate {
public:
    explicit PolicyHistoryGate(std::uint64_t periodUs = 20000)
        : period_us_(periodUs) {}

    bool shouldAppend(std::uint64_t stateSequence, std::uint64_t simTimeUs) const{
        if(!initialized_){
            return true;
        }
        return stateSequence != last_state_sequence_ &&
               simTimeUs > last_sim_time_us_ &&
               simTimeUs - last_sim_time_us_ >= period_us_;
    }

    void commit(std::uint64_t stateSequence, std::uint64_t simTimeUs){
        last_state_sequence_ = stateSequence;
        last_sim_time_us_ = simTimeUs;
        initialized_ = true;
    }

    void reset(){
        last_state_sequence_ = 0;
        last_sim_time_us_ = 0;
        initialized_ = false;
    }

    std::uint64_t lastStateSequence() const { return last_state_sequence_; }
    std::uint64_t lastSimTimeUs() const { return last_sim_time_us_; }

private:
    std::uint64_t period_us_;
    std::uint64_t last_state_sequence_ = 0;
    std::uint64_t last_sim_time_us_ = 0;
    bool initialized_ = false;
};

struct ControlSchedulerInput {
    std::uint64_t current_sim_time_us = 0;
    std::uint64_t current_state_sequence = 0;
    std::uint64_t reset_generation = 0;
};

struct ControlSchedulerDecision {
    bool should_advance = false;
    bool reset_detected = false;
    bool state_repeated = false;
    std::uint64_t accepted_control_time_us = 0;
    std::uint64_t accepted_control_dt_us = 0;
    std::uint64_t accepted_state_sequence = 0;
    std::uint64_t next_deadline_us = 0;
    std::uint64_t lag_us = 0;
    std::uint64_t missed_periods = 0;
};

class ControlTimeScheduler {
public:
    explicit ControlTimeScheduler(std::uint64_t controlPeriodUs)
        : control_period_us_(controlPeriodUs == 0 ? 1 : controlPeriodUs) {}

    ControlSchedulerDecision update(const ControlSchedulerInput &input){
        ControlSchedulerDecision decision;
        if(input.current_sim_time_us == 0 || input.current_state_sequence == 0){
            return decision;
        }

        if(initialized_ &&
           (input.current_sim_time_us < last_observed_sim_time_us_ ||
            input.reset_generation != epoch_generation_)){
            reset();
            decision.reset_detected = true;
        }

        if(!initialized_){
            initialized_ = true;
            epoch_generation_ = input.reset_generation;
            last_observed_sim_time_us_ = input.current_sim_time_us;
            next_deadline_us_ = saturatingAdd(input.current_sim_time_us, control_period_us_);
            decision.next_deadline_us = next_deadline_us_;
            return decision;
        }

        last_observed_sim_time_us_ = input.current_sim_time_us;

        if(input.current_state_sequence == last_accepted_state_sequence_){
            decision.state_repeated = true;
            decision.next_deadline_us = next_deadline_us_;
            return decision;
        }

        if(input.current_sim_time_us < next_deadline_us_){
            decision.next_deadline_us = next_deadline_us_;
            return decision;
        }

        decision.should_advance = true;
        decision.accepted_control_time_us = next_deadline_us_;
        decision.accepted_control_dt_us = control_period_us_;
        decision.accepted_state_sequence = input.current_state_sequence;
        decision.lag_us = input.current_sim_time_us - next_deadline_us_;
        decision.missed_periods = decision.lag_us / control_period_us_;
        last_accepted_state_sequence_ = input.current_state_sequence;
        next_deadline_us_ = saturatingAdd(next_deadline_us_,
                                          saturatingMultiply(decision.missed_periods + 1,
                                                             control_period_us_));
        decision.next_deadline_us = next_deadline_us_;
        return decision;
    }

    void reset(){
        initialized_ = false;
        epoch_generation_ = 0;
        last_observed_sim_time_us_ = 0;
        last_accepted_state_sequence_ = 0;
        next_deadline_us_ = 0;
    }

    std::uint64_t controlPeriodUs() const { return control_period_us_; }
    std::uint64_t nextDeadlineUs() const { return next_deadline_us_; }
    std::uint64_t lastAcceptedStateSequence() const { return last_accepted_state_sequence_; }

private:
    static std::uint64_t saturatingAdd(std::uint64_t a, std::uint64_t b){
        const std::uint64_t max = std::numeric_limits<std::uint64_t>::max();
        return max - a < b ? max : a + b;
    }

    static std::uint64_t saturatingMultiply(std::uint64_t a, std::uint64_t b){
        if(a == 0 || b == 0){
            return 0;
        }
        const std::uint64_t max = std::numeric_limits<std::uint64_t>::max();
        return a > max / b ? max : a * b;
    }

    std::uint64_t control_period_us_;
    bool initialized_ = false;
    std::uint64_t epoch_generation_ = 0;
    std::uint64_t last_observed_sim_time_us_ = 0;
    std::uint64_t last_accepted_state_sequence_ = 0;
    std::uint64_t next_deadline_us_ = 0;
};

#endif  // TIMING_ALIGNMENT_H
