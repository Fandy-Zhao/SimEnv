/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

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
    _ctrlComp->sendRecv();
    _ctrlComp->ioInterFreeDog->sendRecv();

    // Apply ROS /fsm/state_cmd latch (after sendRecv to override keyboard)
    if (_ctrlComp->pendingStateCmd != UserCommand::NONE) {
        _ctrlComp->lowState->userCmd = _ctrlComp->pendingStateCmd;
        _ctrlComp->pendingStateCmd = UserCommand::NONE;
    }
    if(!_ctrlComp->ioInter->hasFullStateFeedback()){
        if(!_waitingForStateFeedback){
            std::cout << "[INFO] Waiting for Gazebo joint state feedback before accepting stand command." << std::endl;
            _waitingForStateFeedback = true;
        }
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    if(_waitingForStateFeedback){
        std::cout << "[INFO] Gazebo joint state feedback is ready." << std::endl;
        _waitingForStateFeedback = false;
    }
    if(!_ctrlComp->lowState->isFinite()){
        std::cout << "[WARNING] Gazebo state feedback is not finite; skipping control update." << std::endl;
        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
        return;
    }
    _ctrlComp->runWaveGen();
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

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
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
