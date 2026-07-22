#ifndef POLICY_SNAPSHOTS_H
#define POLICY_SNAPSHOTS_H

#include <array>
#include <cstdint>
#include <mutex>

#include <ros/time.h>

#include "message/LowlevelState.h"

struct PolicyInputSnapshot {
    LowlevelState low_state;
    std::array<double, 4> base_w_orientation{{0.0, 0.0, 0.0, 1.0}};
    std::array<double, 3> base_w_angular_velocity{{0.0, 0.0, 0.0}};
    bool base_world_valid = false;
    std::uint64_t sim_time_us = 0;
    std::uint64_t state_sequence = 0;
    bool valid = false;
};

struct PolicyCommandSnapshot {
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;
    ros::Time stamp;
    std::uint64_t sequence = 0;
    bool valid = false;
};

struct PolicyOutputSnapshot {
    std::array<float, 12> raw_action{};
    std::array<float, 12> q_target{};
    std::uint64_t action_sequence = 0;
    std::uint64_t source_state_sequence = 0;
    std::uint64_t source_sim_time_us = 0;
    std::uint64_t reset_generation = 0;
    bool valid = false;
};

class PolicyOutputBuffer {
public:
    void publish(const PolicyOutputSnapshot &snapshot){
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = snapshot;
    }

    PolicyOutputSnapshot read() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

    void invalidate(){
        publish(PolicyOutputSnapshot{});
    }

private:
    mutable std::mutex mutex_;
    PolicyOutputSnapshot snapshot_;
};

#endif  // POLICY_SNAPSHOTS_H
