/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include "common/unitreeRobot.h"
#include "Gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include "interface/IOFREEDOGSDK.h"
#include <string>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <ros/time.h>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

struct CtrlComponents{
public:
    CtrlComponents(IOInterface *ioInter, IOFREEDOGSDK *ioInterFreeDog)
        : ioInter(ioInter), ioInterFreeDog(ioInterFreeDog) {
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete robotModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;
#ifdef COMPILE_DEBUG
        delete plot;
#endif  // COMPILE_DEBUG
    }
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;
    IOFREEDOGSDK* ioInterFreeDog;
    QuadrupedRobot *robotModel;
    WaveGenerator *waveGen;
    Estimator *estimator;
    BalanceCtrl *balCtrl;

#ifdef COMPILE_DEBUG
    PyPlot *plot;
#endif  // COMPILE_DEBUG

    VecInt4 *contact;
    Vec4 *phase;

    double dt;
    std::uint64_t controlPeriodUs = 2000;
    double controlDt = 0.0;
    ros::Time controlTime;
    std::uint64_t controlResetGeneration = 0;
    std::uint64_t fsmSequence = 0;
    std::uint64_t waveSequence = 0;
    std::uint64_t gaitCycleSequence = 0;
    std::uint64_t acceptedStateSequence = 0;
    std::uint64_t nextControlDeadlineUs = 0;
    std::uint64_t schedulerLagUs = 0;
    std::uint64_t schedulerMissedPeriods = 0;
    std::uint64_t repeatedStateRejectedCount = 0;
    double resolvedVx = 0.0;
    double resolvedVy = 0.0;
    double resolvedYawRate = 0.0;
    bool *running;
    CtrlPlatform ctrlPlatform;

    // Latch for /fsm/state_cmd ROS topic. Set by ROS callback, applied by
    // FSM::run() after sendRecv() to override keyboard userCmd.
    UserCommand pendingStateCmd = UserCommand::NONE;

    // ---- FSM command-chain diagnostics (G1-F) ----
    std::uint64_t fsmCmdCallbackSequence = 0;
    std::uint64_t fsmTransitionSequence = 0;
    std::uint64_t fsmCmdSimTimeUs = 0;
    int fsmRawRosCmd = 0;
    int fsmMappedUserCmd = 0;
    int fsmKeyboardUserCmd = 0;
    int fsmResolvedUserCmd = 0;
    int fsmCurrentState = 0;
    int fsmRequestedState = 0;
    int fsmNextState = 0;
    int fsmCommandSource = 0;      // 0=NONE, 1=ROS, 2=KEYBOARD, 3=JOYSTICK, 4=SAFETY
    int fsmTransitionResult = 0;   // 0=NONE, 1=ACCEPTED, 2=NO_CMD, 3=WRONG_SRC, 4=GUARD_REJECTED, 5=ALREADY_ACTIVE, 6=DISABLED
    int fsmGuardRejectReason = 0;  // 0=none, 1=height, 2=contact, 3=orientation, 4=safety, 5=other

    // ---- G2-D1 Pre-WAVE block diagnostics (write-only, no control effect) ----
    struct PreWaveDiagnostics {
        // Readiness breakdown
        bool height_ready = false;
        bool stance_ready = false;
        bool contact_ready = false;
        bool contact_fresh[4] = {false, false, false, false};
        float contact_force[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        bool linear_speed_ready = false;
        bool angular_speed_ready = false;
        bool tilt_ready = false;
        bool readiness_met = false;
        bool readiness_hold_complete = false;
        double readiness_hold_elapsed = 0.0;
        // Wave diagnostics
        bool wave_start_requested = false;
        std::uint64_t wave_start_sequence = 0;
        std::uint64_t wave_all_entered_sequence = 0;
        bool wave_cancel_requested = false;
        std::uint64_t wave_cancel_sequence = 0;
        int wave_cancel_reason = 0;  // 0=none, 1=nonfinite_state, 2=nonfinite_cmd, 3=nonfinite_output, 4=attitude, 5=contact_loss, 6=time_reset
        // First block latch
        int first_block_reason = 0;  // 0=PRE_WAVE_BLOCK_NONE
        std::uint64_t first_block_sim_time_us = 0;
        std::uint64_t first_block_control_sequence = 0;
        // Fall
        double model_height = 0.0;
        bool fall_predicate = false;
        std::uint64_t first_fall_sim_time_us = 0;
        // Guards
        bool numerical_guard_triggered = false;
        int numerical_guard_stage = 0;  // 1=state, 2=command, 3=output
        bool safety_guard_triggered = false;
        // FSM Trotting
        bool trotting_entered = false;
        std::uint64_t trotting_enter_sequence = 0;
        std::uint64_t trotting_exit_sequence = 0;
        // Additional diagnostic values
        double linear_speed = 0.0;
        double angular_speed = 0.0;
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
    } preWave;

    void sendRecv(){
        ioInter->sendRecv(lowCmd, lowState);
    }

    void recvStateOnly(){
        ioInter->recvStateOnly(lowState);
    }

    void publishCmdOnly(){
        ioInter->publishCmdOnly(lowCmd);
    }

    void runWaveGen(){
        const double previousPhase0 = (*phase)(0);
        const bool hadPreviousPhase = _gaitPhaseInitialized;
        waveGen->calcContactPhase(*phase, *contact, _waveStatus, controlTime);
        ++waveSequence;
        if(_waveStatus == WaveStatus::WAVE_ALL){
            if(hadPreviousPhase && previousPhase0 > 0.8 && (*phase)(0) < 0.2){
                ++gaitCycleSequence;
            }
            _gaitPhaseInitialized = true;
        } else {
            _gaitPhaseInitialized = false;
        }
    }

    double getControlDt() const{
        return std::isfinite(controlDt) && controlDt > 0.0 ? controlDt : dt;
    }

    void resetWaveTime(const ros::Time &time){
        waveGen->resetTime(time, _waveStatus);
        _gaitPhaseInitialized = false;
    }

    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllStanceNow(){
        _waveStatus = WaveStatus::STANCE_ALL;
        contact->setOnes();
        phase->setConstant(0.5);
        waveGen->resetTime(controlTime, _waveStatus);
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;
    }

    WaveStatus waveStatus() const{
        return _waveStatus;
    }

    void geneObj(){
        estimator = new Estimator(robotModel, lowState, contact, phase, dt);
        balCtrl = new BalanceCtrl(robotModel);

#ifdef COMPILE_DEBUG
        plot = new PyPlot();
        balCtrl->setPyPlot(plot);
        estimator->setPyPlot(plot);
#endif  // COMPILE_DEBUG
    }

private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;
    bool _gaitPhaseInitialized = false;
};

#endif  // CTRLCOMPONENTS_H
