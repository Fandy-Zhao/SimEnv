/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include "common/TimingDiagnostics.h"
#include <cmath>
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp),
     _controlScheduler(ctrlComp->controlPeriodUs){

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
    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        _ctrlComp->recvStateOnly();
    }else{
        _ctrlComp->sendRecv();
    }
    _ctrlComp->ioInterFreeDog->sendRecv();

    if(!_ctrlComp->ioInter->hasFullStateFeedback()){
        if(!_waitingForStateFeedback){
            std::cout << "[INFO] Waiting for Gazebo joint state feedback before accepting stand command." << std::endl;
            _waitingForStateFeedback = true;
        }
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        recordTiming(false, false);
        return;
    }
    if(_waitingForStateFeedback){
        std::cout << "[INFO] Gazebo joint state feedback is ready." << std::endl;
        _waitingForStateFeedback = false;
    }
    if(!_ctrlComp->lowState->isFinite()){
        std::cout << "[WARNING] Gazebo state feedback is not finite; skipping control update." << std::endl;
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        recordTiming(false, false);
        return;
    }
    const std::uint64_t acceptedStateSequence = _ctrlComp->acceptedStateSequence;
    const bool repeatedStateConsumed =
        _lastAcceptedStateSequence != 0 && acceptedStateSequence == _lastAcceptedStateSequence;
    _lastAcceptedStateSequence = acceptedStateSequence;
    if(!updateControlTime()){
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        recordTiming(false, repeatedStateConsumed);
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

    if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
        _ctrlComp->publishCmdOnly();
    }
    recordTiming(true, repeatedStateConsumed);
    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
}

void FSM::recordTiming(bool accepted, bool repeatedStateConsumed){
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
    timing.state_sequence = _ctrlComp->acceptedStateSequence;
    timing.state_stamp_us = _ctrlComp->ioInter->stateStampUs();
    timing.fsm_iteration = _fsmIteration;
    timing.fsm_sequence = _ctrlComp->fsmSequence;
    timing.estimator_sequence = _ctrlComp->estimator ? _ctrlComp->estimator->runSequence() : 0;
    timing.estimator_source_state_sequence = accepted ? timing.state_sequence : 0;
    timing.wave_sequence = _ctrlComp->waveSequence;
    timing.gait_cycle_sequence = _ctrlComp->gaitCycleSequence;
    timing.reset_generation = _ctrlComp->controlResetGeneration;
    timing.runtime_ctrl_dt_us = _ctrlComp->controlPeriodUs;
    timing.target_control_hz = _ctrlComp->controlPeriodUs > 0 ?
        1000000.0 / static_cast<double>(_ctrlComp->controlPeriodUs) : 0.0;
    timing.control_update_accepted = accepted;
    timing.new_state_consumed = accepted && !repeatedStateConsumed;
    timing.repeated_state_consumed = accepted && repeatedStateConsumed;
    timing.scheduler_accepted = accepted;
    timing.accepted_state_sequence = _ctrlComp->acceptedStateSequence;
    timing.scheduler_lag_us = _ctrlComp->schedulerLagUs;
    timing.missed_periods = _ctrlComp->schedulerMissedPeriods;
    timing.repeated_state_rejected_count = _ctrlComp->repeatedStateRejectedCount;
    timing.fsm_state = _currentState != nullptr ? static_cast<int>(_currentState->_stateName) : 0;
    timing.wave_status = static_cast<int>(_ctrlComp->waveStatus());
    timing.resolved_vx = _ctrlComp->resolvedVx;
    timing.resolved_vy = _ctrlComp->resolvedVy;
    timing.resolved_yaw_rate = _ctrlComp->resolvedYawRate;
    for(int i = 0; i < 4; ++i){
        timing.phase[i] = (*_ctrlComp->phase)(i);
        timing.contact[i] = (*_ctrlComp->contact)(i);
    }
    diagnostics.record(timing);
}

bool FSM::updateControlTime(){
    if(_ctrlComp->ctrlPlatform != CtrlPlatform::GAZEBO){
        _ctrlComp->controlDt = _ctrlComp->dt;
        _ctrlComp->controlTime = ros::Time::now();
        _ctrlComp->acceptedStateSequence = _ctrlComp->ioInter->stateSequence();
        return true;
    }

    const std::uint64_t simTimeUs = _ctrlComp->ioInter->stateStampUs();
    const std::uint64_t stateSequence = _ctrlComp->ioInter->stateSequence();
    if(simTimeUs == 0 || stateSequence == 0){
        ROS_WARN_THROTTLE(1.0, "Waiting for non-zero Gazebo /clock before running control updates.");
        return false;
    }
    ros::Time now;
    now.fromNSec(simTimeUs * 1000ULL);

    if(!_simClockInitialized){
        _lastSimTime = now;
        _lastObservedSimTimeUs = simTimeUs;
        _ctrlComp->controlTime = now;
        _ctrlComp->controlDt = 0.0;
        _ctrlComp->setAllStance();
        _ctrlComp->resetWaveTime(now);
        _controlScheduler.reset();
        const auto decision = _controlScheduler.update(
            {simTimeUs, stateSequence, _ctrlComp->controlResetGeneration});
        _ctrlComp->nextControlDeadlineUs = decision.next_deadline_us;
        _simClockInitialized = true;
        return false;
    }

    if(simTimeUs == _lastObservedSimTimeUs){
        return false;
    }

    if(simTimeUs < _lastObservedSimTimeUs){
        _lastObservedSimTimeUs = simTimeUs;
        resetForTimeDiscontinuity("Gazebo simulation time moved backward", now,
                                  ControlTimeResetReason::MovedBackward);
        _controlScheduler.reset();
        const auto decision = _controlScheduler.update(
            {simTimeUs, stateSequence, _ctrlComp->controlResetGeneration});
        _ctrlComp->nextControlDeadlineUs = decision.next_deadline_us;
        return false;
    }

    const double simDt = static_cast<double>(simTimeUs - _lastObservedSimTimeUs) / 1000000.0;
    _lastObservedSimTimeUs = simTimeUs;
    _lastSimTime = now;
    _simPauseHandled = false;

    if(simDt > _maxSimTimeStep){
        resetForTimeDiscontinuity("Gazebo simulation time jumped forward", now,
                                  ControlTimeResetReason::JumpedForward);
        _controlScheduler.reset();
        const auto decision = _controlScheduler.update(
            {simTimeUs, stateSequence, _ctrlComp->controlResetGeneration});
        _ctrlComp->nextControlDeadlineUs = decision.next_deadline_us;
        return false;
    }

    const auto decision = _controlScheduler.update(
        {simTimeUs, stateSequence, _ctrlComp->controlResetGeneration});
    _ctrlComp->nextControlDeadlineUs = decision.next_deadline_us;
    _ctrlComp->schedulerLagUs = decision.lag_us;
    _ctrlComp->schedulerMissedPeriods = decision.missed_periods;
    if(decision.state_repeated){
        ++_ctrlComp->repeatedStateRejectedCount;
    }
    if(!decision.should_advance){
        return false;
    }

    ros::Time acceptedTime;
    acceptedTime.fromNSec(decision.accepted_control_time_us * 1000ULL);
    _ctrlComp->controlTime = acceptedTime;
    _ctrlComp->controlDt = static_cast<double>(decision.accepted_control_dt_us) / 1000000.0;
    _ctrlComp->acceptedStateSequence = decision.accepted_state_sequence;
    return true;
}

void FSM::resetForTimeDiscontinuity(const char *reason, const ros::Time &now,
                                    ControlTimeResetReason resetReason){
    ROS_WARN("%s at %.6f s; resetting gait time and holding all stance.",
             reason, now.toSec());
    _ctrlComp->controlTime = now;
    _ctrlComp->controlDt = 0.0;
    ++_ctrlComp->controlResetGeneration;
    _ctrlComp->acceptedStateSequence = 0;
    _ctrlComp->nextControlDeadlineUs = 0;
    _ctrlComp->schedulerLagUs = 0;
    _ctrlComp->schedulerMissedPeriods = 0;
    _ctrlComp->setAllStance();
    _ctrlComp->resetWaveTime(now);
    if(_ctrlComp->estimator != nullptr){
        _ctrlComp->estimator->resetState();
    }
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
