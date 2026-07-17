/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSM_H
#define FSM_H

// FSM States
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FreeStand.h"
#ifndef UNITREE_DISABLE_TORCH_POLICY
#include "FSM/State_Trotting.h"
#endif  // UNITREE_DISABLE_TORCH_POLICY
#include "FSM/State_BalanceTest.h"
#include "FSM/State_SwingTest.h"
#include "FSM/State_StepTest.h"
#include <cstdint>
#include "common/enumClass.h"
#include "common/TimingAlignment.h"
#include "control/CtrlComponents.h"
#include <ros/ros.h>
#ifdef COMPILE_WITH_MOVE_BASE
    #include "FSM/State_move_base.h"
#endif  // COMPILE_WITH_MOVE_BASE
#ifndef UNITREE_DISABLE_TORCH_POLICY
#include "FSM/State_RL_test.h"
#endif  // UNITREE_DISABLE_TORCH_POLICY

struct FSMStateList{
    FSMState *invalid;
    State_Passive *passive;
    State_FixedStand *fixedStand;
    State_FreeStand *freeStand;
#ifndef UNITREE_DISABLE_TORCH_POLICY
    State_Trotting *trotting;
#endif  // UNITREE_DISABLE_TORCH_POLICY
    State_BalanceTest *balanceTest;
    State_SwingTest *swingTest;
    State_StepTest *stepTest;
#ifdef COMPILE_WITH_MOVE_BASE
    State_move_base *moveBase;
#endif  // COMPILE_WITH_MOVE_BASE
#ifndef UNITREE_DISABLE_TORCH_POLICY
    State_RL *rl;
#endif  // UNITREE_DISABLE_TORCH_POLICY

    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        delete freeStand;
#ifndef UNITREE_DISABLE_TORCH_POLICY
        delete trotting;
#endif  // UNITREE_DISABLE_TORCH_POLICY
        delete balanceTest;
        delete swingTest;
        delete stepTest;
#ifdef COMPILE_WITH_MOVE_BASE
        delete moveBase;
#endif  // COMPILE_WITH_MOVE_BASE
#ifndef UNITREE_DISABLE_TORCH_POLICY
        delete rl;
#endif  // UNITREE_DISABLE_TORCH_POLICY
    }
};

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();
private:
    FSMState* getNextState(FSMStateName stateName);
    bool checkSafty();
    bool updateControlTime();
    void resetForTimeDiscontinuity(const char *reason, const ros::Time &now,
                                   ControlTimeResetReason resetReason);
    void recordTiming(bool accepted, bool repeatedStateConsumed);
    CtrlComponents *_ctrlComp;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
    bool _waitingForStateFeedback = false;
    ros::NodeHandle _nh;
    ros::Time _lastSimTime;
    ros::WallTime _lastSimAdvanceWallTime;
    double _maxSimTimeStep = 0.05;
    double _simPauseResetTimeout = 0.5;
    bool _simClockInitialized = false;
    bool _simPauseHandled = false;
    std::uint64_t _fsmIteration = 0;
    std::uint64_t _lastAcceptedStateSequence = 0;
    std::uint64_t _lastObservedSimTimeUs = 0;
    ControlTimeScheduler _controlScheduler;
    ros::Time _diagnosticLastSimTime;
    ros::WallTime _diagnosticLastWallTime;
};


#endif  // FSM_H
