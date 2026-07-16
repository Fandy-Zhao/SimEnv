/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include "common/TimingDiagnostics.h"
#include <cmath>
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    TimingDiagnostics::instance().configure(_nh);

    _nh.param("fsm_max_sim_time_step", _maxSimTimeStep, 0.05);
    _nh.param("fsm_sim_pause_reset_timeout", _simPauseResetTimeout, 0.5);
    if(!std::isfinite(_maxSimTimeStep) || _maxSimTimeStep < 0.002 ||
       _maxSimTimeStep > 1.0){
        ROS_WARN("Invalid fsm_max_sim_time_step; using 0.05 seconds.");
        _maxSimTimeStep = 0.05;
    }
    if(!std::isfinite(_simPauseResetTimeout) || _simPauseResetTimeout < 0.1 ||
       _simPauseResetTimeout > 10.0){
        ROS_WARN("Invalid fsm_sim_pause_reset_timeout; using 0.5 wall seconds.");
        _simPauseResetTimeout = 0.5;
    }

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
#ifndef UNITREE_DISABLE_TORCH_POLICY
    _stateList.trotting = new State_Trotting(_ctrlComp);
#endif  // UNITREE_DISABLE_TORCH_POLICY
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);
#endif  // COMPILE_WITH_MOVE_BASE
#ifndef UNITREE_DISABLE_TORCH_POLICY
    _stateList.rl = new State_RL(_ctrlComp);
#endif  // UNITREE_DISABLE_TORCH_POLICY
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run(){
    _startTime = getSystemTime();
    ++_fsmIteration;
    _ctrlComp->sendRecv();
    _ctrlComp->ioInterFreeDog->sendRecv();

    if(!_ctrlComp->ioInter->hasFullStateFeedback()){
        if(!_waitingForStateFeedback){
            std::cout << "[INFO] Waiting for Gazebo joint state feedback before accepting stand command." << std::endl;
            _waitingForStateFeedback = true;
        }
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        recordTiming(false);
        return;
    }
    if(_waitingForStateFeedback){
        std::cout << "[INFO] Gazebo joint state feedback is ready." << std::endl;
        _waitingForStateFeedback = false;
    }
    if(!_ctrlComp->lowState->isFinite()){
        std::cout << "[WARNING] Gazebo state feedback is not finite; skipping control update." << std::endl;
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        recordTiming(false);
        return;
    }
    if(!updateControlTime()){
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        recordTiming(false);
        return;
    }

    // Apply ROS /fsm/state_cmd only on an advancing simulation step so a
    // command received while Gazebo is paused remains latched until it can run.
    if (_ctrlComp->pendingStateCmd != UserCommand::NONE) {
        _ctrlComp->lowState->userCmd = _ctrlComp->pendingStateCmd;
        _ctrlComp->pendingStateCmd = UserCommand::NONE;
    }
    _ctrlComp->runWaveGen();
    _ctrlComp->estimator->setDt(_ctrlComp->getControlDt());
    _ctrlComp->estimator->run();
    if(!checkSafty()){
        // _ctrlComp->ioInter->setPassive();
    }

    if(_mode == FSMMode::NORMAL){
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _nextState = getNextState(_nextStateName);
            if(_nextState != nullptr){
                _mode = FSMMode::CHANGE;
                std::cout << "Switched from " << _currentState->_stateNameString
                          << " to " << _nextState->_stateNameString << std::endl;
            } else {
                std::cerr << "[WARNING] FSM: requested state (enum "
                          << static_cast<int>(_nextStateName)
                          << ") is not available (disabled at build time). Ignoring." << std::endl;
            }
        }
    }
    else if(_mode == FSMMode::CHANGE){
        if(_nextState == nullptr){
            _mode = FSMMode::NORMAL;
        } else {
            _currentState->exit();
            _currentState = _nextState;
            _currentState->enter();
            _mode = FSMMode::NORMAL;
            _currentState->run();
        }
    }

    recordTiming(true);
    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

void FSM::recordTiming(bool accepted){
    TimingDiagnostics &diagnostics = TimingDiagnostics::instance();
    if(!diagnostics.enabled()){
        return;
    }
    const ros::Time simNow = ros::Time::now();
    const ros::WallTime wallNow = ros::WallTime::now();
    double estimatedRtf = 0.0;
    if(!_diagnosticLastSimTime.isZero() && !_diagnosticLastWallTime.isZero() &&
       simNow > _diagnosticLastSimTime && wallNow > _diagnosticLastWallTime){
        estimatedRtf = (simNow - _diagnosticLastSimTime).toSec() /
                       (wallNow - _diagnosticLastWallTime).toSec();
    }
    if(simNow != _diagnosticLastSimTime){
        _diagnosticLastSimTime = simNow;
        _diagnosticLastWallTime = wallNow;
    }
    TimingRecord timing;
    timing.event = "FSM";
    timing.wall_time_ns = wallNow.toNSec();
    timing.sim_time_us = simNow.toNSec() / 1000U;
    timing.sim_dt_us = accepted ? static_cast<std::int64_t>(_ctrlComp->controlDt * 1e6) : 0;
    timing.estimated_rtf = estimatedRtf;
    timing.state_sequence = _ctrlComp->ioInter->stateSequence();
    timing.state_stamp_us = _ctrlComp->ioInter->stateStampUs();
    timing.fsm_iteration = _fsmIteration;
    timing.control_update_accepted = accepted;
    diagnostics.record(timing);
}

bool FSM::updateControlTime(){
    if(_ctrlComp->ctrlPlatform != CtrlPlatform::GAZEBO){
        _ctrlComp->controlDt = _ctrlComp->dt;
        _ctrlComp->controlTime = ros::Time::now();
        return true;
    }

    const ros::Time now = ros::Time::now();
    const ros::WallTime wallNow = ros::WallTime::now();
    if(now.isZero()){
        ROS_WARN_THROTTLE(1.0, "Waiting for non-zero Gazebo /clock before running control updates.");
        return false;
    }

    if(!_simClockInitialized){
        _lastSimTime = now;
        _lastSimAdvanceWallTime = wallNow;
        _ctrlComp->controlTime = now;
        _ctrlComp->controlDt = 0.0;
        _ctrlComp->setAllStance();
        _ctrlComp->resetWaveTime(now);
        _simClockInitialized = true;
        return false;
    }

    if(now == _lastSimTime){
        const double stoppedWallTime = (wallNow - _lastSimAdvanceWallTime).toSec();
        if(!_simPauseHandled && stoppedWallTime >= _simPauseResetTimeout){
            resetForTimeDiscontinuity("Gazebo simulation time paused", now,
                                      ControlTimeResetReason::Paused);
            _simPauseHandled = true;
        }
        return false;
    }

    const double simDt = (now - _lastSimTime).toSec();
    _lastSimTime = now;
    _lastSimAdvanceWallTime = wallNow;
    _simPauseHandled = false;

    if(!std::isfinite(simDt) || simDt <= 0.0){
        resetForTimeDiscontinuity("Gazebo simulation time moved backward", now,
                                  ControlTimeResetReason::MovedBackward);
        return false;
    }
    if(simDt > _maxSimTimeStep){
        resetForTimeDiscontinuity("Gazebo simulation time jumped forward", now,
                                  ControlTimeResetReason::JumpedForward);
        return false;
    }

    _ctrlComp->controlTime = now;
    _ctrlComp->controlDt = simDt;
    return true;
}

void FSM::resetForTimeDiscontinuity(const char *reason, const ros::Time &now,
                                    ControlTimeResetReason resetReason){
    ROS_WARN("%s at %.6f s; resetting gait time and holding all stance.",
             reason, now.toSec());
    _ctrlComp->controlTime = now;
    _ctrlComp->controlDt = 0.0;
    _ctrlComp->setAllStance();
    _ctrlComp->resetWaveTime(now);
    if(_currentState != nullptr){
        _currentState->onControlTimeReset(resetReason);
    }
}

FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
#ifndef UNITREE_DISABLE_TORCH_POLICY
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
#endif  // UNITREE_DISABLE_TORCH_POLICY
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
#ifndef UNITREE_DISABLE_TORCH_POLICY
    case FSMStateName::RL:
        return _stateList.rl;
    break;
#endif  // UNITREE_DISABLE_TORCH_POLICY
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}
