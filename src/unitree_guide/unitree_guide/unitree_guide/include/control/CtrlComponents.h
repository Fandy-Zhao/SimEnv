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
