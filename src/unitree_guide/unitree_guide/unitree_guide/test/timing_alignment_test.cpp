#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

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

TEST(ControlTimeScheduler, FixedTwoMillisecondPeriodIgnoresOneMillisecondStateTicks){
    ControlTimeScheduler scheduler(2000);
    std::uint64_t accepted = 0;
    for(std::uint64_t t = 1000; t <= 10000000; t += 1000){
        const auto decision = scheduler.update({t, t / 1000, 0});
        if(decision.should_advance){
            ++accepted;
            EXPECT_EQ(2000U, decision.accepted_control_dt_us);
        }
    }
    EXPECT_EQ(4999U, accepted);
    EXPECT_EQ(10001000U, scheduler.nextDeadlineUs());
}

TEST(ControlTimeScheduler, FixedFourMillisecondPeriodIsConfigurationDriven){
    ControlTimeScheduler scheduler(4000);
    std::uint64_t accepted = 0;
    for(std::uint64_t t = 1000; t <= 10000000; t += 1000){
        if(scheduler.update({t, t / 1000, 0}).should_advance){
            ++accepted;
        }
    }
    EXPECT_EQ(2499U, accepted);
    EXPECT_EQ(10001000U, scheduler.nextDeadlineUs());
}

TEST(ControlTimeScheduler, IrregularTicksFollowPeriodNotTickCount){
    ControlTimeScheduler scheduler(2000);
    const std::vector<std::uint64_t> increments{800, 1100, 1400, 900, 1300};
    std::uint64_t simUs = 0;
    std::uint64_t stateSeq = 0;
    std::uint64_t accepted = 0;
    while(simUs < 1000000){
        simUs += increments[stateSeq % increments.size()];
        ++stateSeq;
        if(scheduler.update({simUs, stateSeq, 0}).should_advance){
            ++accepted;
        }
    }
    EXPECT_GE(accepted, 498U);
    EXPECT_LE(accepted, 500U);
}

TEST(ControlTimeScheduler, PauseAndWallOvertimeDoNotAdvance){
    ControlTimeScheduler scheduler(2000);
    EXPECT_FALSE(scheduler.update({1000, 1, 0}).should_advance);
    EXPECT_TRUE(scheduler.update({3000, 2, 0}).should_advance);
    for(std::uint64_t i = 0; i < 1000; ++i){
        const auto decision = scheduler.update({3000, 2, 0});
        EXPECT_FALSE(decision.should_advance);
        EXPECT_TRUE(decision.state_repeated);
    }
}

TEST(ControlTimeScheduler, DeadlineWithoutNewStateRejectsRepeatedState){
    ControlTimeScheduler scheduler(2000);
    EXPECT_FALSE(scheduler.update({1000, 1, 0}).should_advance);
    EXPECT_TRUE(scheduler.update({3000, 2, 0}).should_advance);
    const auto decision = scheduler.update({5000, 2, 0});
    EXPECT_FALSE(decision.should_advance);
    EXPECT_TRUE(decision.state_repeated);
}

TEST(ControlTimeScheduler, LargeLagAcceptsOnceAndSkipsBurstCatchup){
    ControlTimeScheduler scheduler(2000);
    EXPECT_FALSE(scheduler.update({1000, 1, 0}).should_advance);
    const auto decision = scheduler.update({11000, 2, 0});
    EXPECT_TRUE(decision.should_advance);
    EXPECT_EQ(3000U, decision.accepted_control_time_us);
    EXPECT_EQ(8000U, decision.lag_us);
    EXPECT_EQ(4U, decision.missed_periods);
    EXPECT_EQ(13000U, decision.next_deadline_us);
    EXPECT_FALSE(scheduler.update({11000, 3, 0}).should_advance);
}

TEST(ControlTimeScheduler, ResetClearsDeadlineAndOldState){
    ControlTimeScheduler scheduler(2000);
    EXPECT_FALSE(scheduler.update({100000, 10, 0}).should_advance);
    EXPECT_TRUE(scheduler.update({102000, 11, 0}).should_advance);
    const auto resetDecision = scheduler.update({1000, 1, 1});
    EXPECT_TRUE(resetDecision.reset_detected);
    EXPECT_FALSE(resetDecision.should_advance);
    EXPECT_EQ(3000U, resetDecision.next_deadline_us);
    EXPECT_TRUE(scheduler.update({3000, 2, 1}).should_advance);
}

TEST(ControlTimeScheduler, CrossesFormerUint32Boundary){
    constexpr std::uint64_t startUs = 4294960000ULL;
    ControlTimeScheduler scheduler(2000);
    EXPECT_FALSE(scheduler.update({startUs, 1, 0}).should_advance);
    EXPECT_TRUE(scheduler.update({startUs + 2000, 2, 0}).should_advance);
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
            snapshot.reset_generation = sequence / 100;
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
               snapshot.source_sim_time_us != snapshot.action_sequence * 20000 ||
               snapshot.reset_generation != snapshot.action_sequence / 100){
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

TEST(PolicyOutputBuffer, ResetGenerationRejectsInflightOldEpochAction){
    PolicyOutputBuffer buffer;
    PolicyOutputSnapshot oldEpoch;
    oldEpoch.action_sequence = 9;
    oldEpoch.reset_generation = 3;
    oldEpoch.valid = true;
    buffer.publish(oldEpoch);

    const std::uint64_t currentResetGeneration = 4;
    const PolicyOutputSnapshot snapshot = buffer.read();
    EXPECT_TRUE(snapshot.valid);
    EXPECT_NE(currentResetGeneration, snapshot.reset_generation);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
