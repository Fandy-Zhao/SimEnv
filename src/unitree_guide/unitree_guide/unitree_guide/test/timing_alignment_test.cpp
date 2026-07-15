#include <atomic>
#include <cstdint>
#include <thread>

#include <gtest/gtest.h>

#include "common/TimingAlignment.h"
#include "interface/PolicySnapshots.h"

TEST(PolicyWaitClassification, CoversAdvanceOvertimeResetAndShutdown){
    EXPECT_EQ(PolicyWaitExitReason::SimPeriodReached,
              classifyPolicyWaitExit(1020000, 1000000, 20000, false));
    EXPECT_EQ(PolicyWaitExitReason::WallOvertime,
              classifyPolicyWaitExit(1000000, 1000000, 20000, false));
    EXPECT_EQ(PolicyWaitExitReason::SimTimeReset,
              classifyPolicyWaitExit(1000, 5000000, 20000, false));
    EXPECT_EQ(PolicyWaitExitReason::Shutdown,
              classifyPolicyWaitExit(1020000, 1000000, 20000, true));
}

TEST(PolicyWaitClassification, CrossesFormerUint32MicrosecondBoundary){
    constexpr std::uint64_t startUs = 4294960000ULL;
    EXPECT_EQ(PolicyWaitExitReason::SimPeriodReached,
              classifyPolicyWaitExit(startUs + 20000ULL, startUs, 20000, false));
}

TEST(PolicyHistoryGate, RejectsDuplicateEarlyAndBackwardState){
    PolicyHistoryGate gate(20000);
    constexpr std::uint64_t nearUint32Boundary = 4294960000ULL;

    ASSERT_TRUE(gate.shouldAppend(1, nearUint32Boundary));
    gate.commit(1, nearUint32Boundary);
    EXPECT_FALSE(gate.shouldAppend(1, nearUint32Boundary + 20000));
    EXPECT_FALSE(gate.shouldAppend(2, nearUint32Boundary + 19999));
    EXPECT_FALSE(gate.shouldAppend(2, 1000));
    EXPECT_TRUE(gate.shouldAppend(2, nearUint32Boundary + 20000));

    gate.commit(2, nearUint32Boundary + 20000);
    EXPECT_EQ(2U, gate.lastStateSequence());
    EXPECT_EQ(nearUint32Boundary + 20000, gate.lastSimTimeUs());
    gate.reset();
    EXPECT_TRUE(gate.shouldAppend(1, 1000));
}

TEST(PolicyOutputBuffer, ConcurrentReadsObserveOneCompleteGeneration){
    PolicyOutputBuffer buffer;
    std::atomic_bool writerDone{false};
    std::atomic_bool inconsistent{false};

    std::thread writer([&] {
        for(std::uint64_t sequence = 1; sequence <= 10000; ++sequence){
            PolicyOutputSnapshot snapshot;
            snapshot.action_sequence = sequence;
            snapshot.source_state_sequence = sequence * 10;
            snapshot.source_sim_time_us = sequence * 20000;
            snapshot.valid = true;
            snapshot.raw_action.fill(static_cast<float>(sequence));
            snapshot.q_target.fill(static_cast<float>(sequence));
            buffer.publish(snapshot);
        }
        writerDone.store(true, std::memory_order_release);
    });

    std::thread reader([&] {
        while(!writerDone.load(std::memory_order_acquire)){
            const PolicyOutputSnapshot snapshot = buffer.read();
            if(!snapshot.valid){
                continue;
            }
            const float expected = static_cast<float>(snapshot.action_sequence);
            if(snapshot.source_state_sequence != snapshot.action_sequence * 10 ||
               snapshot.source_sim_time_us != snapshot.action_sequence * 20000){
                inconsistent.store(true, std::memory_order_release);
                break;
            }
            for(std::size_t i=0; i<snapshot.raw_action.size(); ++i){
                if(snapshot.raw_action[i] != expected || snapshot.q_target[i] != expected){
                    inconsistent.store(true, std::memory_order_release);
                    return;
                }
            }
        }
    });

    writer.join();
    reader.join();
    EXPECT_FALSE(inconsistent.load(std::memory_order_acquire));
    EXPECT_EQ(10000U, buffer.read().action_sequence);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
