#ifndef TIMING_ALIGNMENT_H
#define TIMING_ALIGNMENT_H

#include <cstdint>

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

#endif  // TIMING_ALIGNMENT_H
