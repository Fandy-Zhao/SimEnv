/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <cmath>
#include <iomanip>

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"),
              _est(ctrlComp->estimator), _phase(ctrlComp->phase),
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel),
              _balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);
    _cmdVelSub = _nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
        &State_Trotting::cmdVelCallback, this);
    _nh.param("trotting_cmd_vel_timeout", _cmdVelTimeout, 0.5);
    if(!std::isfinite(_cmdVelTimeout) || _cmdVelTimeout < 0.1 || _cmdVelTimeout > 5.0){
        ROS_WARN("Invalid trotting_cmd_vel_timeout; using 0.5 seconds.");
        _cmdVelTimeout = 0.5;
    }
    _nh.param("trotting_height_transition_duration", _heightTransitionDuration, 0.75);
    _nh.param("trotting_ready_hold_duration", _readinessHoldDuration, 0.2);
    _nh.param("trotting_ready_linear_velocity", _readyLinearVelocity, 0.12);
    _nh.param("trotting_ready_angular_velocity", _readyAngularVelocity, 0.35);
    _nh.param("trotting_ready_tilt", _readyTilt, 0.17453292519943295);
    _nh.param("trotting_minimum_contact_force", _minimumContactForce, 1.0);
    _nh.param("trotting_wave_abort_tilt", _waveAbortTilt, 0.3490658503988659);
    _nh.param("trotting_wave_contact_loss_duration", _waveContactLossDuration, 0.08);
    if(!std::isfinite(_heightTransitionDuration) || _heightTransitionDuration < 0.5 ||
       _heightTransitionDuration > 1.0){
        ROS_WARN("Invalid trotting_height_transition_duration; using 0.75 seconds.");
        _heightTransitionDuration = 0.75;
    }
    if(!std::isfinite(_readinessHoldDuration) || _readinessHoldDuration < 0.05 ||
       _readinessHoldDuration > 2.0){
        ROS_WARN("Invalid trotting_ready_hold_duration; using 0.2 seconds.");
        _readinessHoldDuration = 0.2;
    }
    if(!std::isfinite(_readyLinearVelocity) || _readyLinearVelocity <= 0.0 ||
       _readyLinearVelocity > 1.0){
        ROS_WARN("Invalid trotting_ready_linear_velocity; using 0.12 m/s.");
        _readyLinearVelocity = 0.12;
    }
    if(!std::isfinite(_readyAngularVelocity) || _readyAngularVelocity <= 0.0 ||
       _readyAngularVelocity > 2.0){
        ROS_WARN("Invalid trotting_ready_angular_velocity; using 0.35 rad/s.");
        _readyAngularVelocity = 0.35;
    }
    if(!std::isfinite(_readyTilt) || _readyTilt <= 0.0 || _readyTilt > 0.7854){
        ROS_WARN("Invalid trotting_ready_tilt; using 10 degrees.");
        _readyTilt = 0.17453292519943295;
    }
    if(!std::isfinite(_minimumContactForce) || _minimumContactForce < 0.0 ||
       _minimumContactForce > 1000.0){
        ROS_WARN("Invalid trotting_minimum_contact_force; using 1 N.");
        _minimumContactForce = 1.0;
    }
    if(!std::isfinite(_waveAbortTilt) || _waveAbortTilt <= _readyTilt ||
       _waveAbortTilt > 1.0472){
        ROS_WARN("Invalid trotting_wave_abort_tilt; using 20 degrees.");
        _waveAbortTilt = 0.3490658503988659;
    }
    if(!std::isfinite(_waveContactLossDuration) ||
       _waveContactLossDuration < 0.02 || _waveContactLossDuration > 1.0){
        ROS_WARN("Invalid trotting_wave_contact_loss_duration; using 0.08 seconds.");
        _waveContactLossDuration = 0.08;
    }

    _gaitHeight = 0.08;

#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

    _vxLim << -0.8, 0.8;
    _vyLim << -0.6, 0.6;
    _wyawLim << -1.0, 1.0;
    resetCommandState();
}

State_Trotting::~State_Trotting(){
    ampthreadRunning = State_Trotting::STOP;
    if(amp_obs_thread != nullptr){
        if(amp_obs_thread->joinable()){
            amp_obs_thread->join();
        }
        delete amp_obs_thread;
        amp_obs_thread = nullptr;
    }
    delete _gait;
}

void State_Trotting::enter(){
    _pcd = _est->getPosition();
    _posFeetGlobalGoal = _est->getFeetPos();
    _velFeetGlobalGoal.setZero();
    _heightTransitionStart = _pcd(2);
    _heightTransitionTarget = -_robModel->getFeetPosIdeal()(2, 0);
    _heightTransitionElapsed = 0.0;
    _readinessStableElapsed = 0.0;
    _waveContactLossElapsed = 0.0;
    _heightTransitionComplete = false;
    _waveReady = false;
    _waveStarted = false;
    _waveAbortLatched = false;
    resetCommandState();
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);

    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllStanceNow();
    _gait->restart();

    // ---- G2-D1: initialize pre-WAVE diagnostics ----
    auto &d = _ctrlComp->preWave;
    d.height_ready = false;
    d.stance_ready = false;
    d.contact_ready = false;
    for(int i=0; i<4; ++i){ d.contact_fresh[i] = false; d.contact_force[i] = 0.0f; }
    d.linear_speed_ready = false;
    d.angular_speed_ready = false;
    d.tilt_ready = false;
    d.readiness_met = false;
    d.readiness_hold_complete = false;
    d.readiness_hold_elapsed = 0.0;
    d.wave_start_requested = false;
    d.wave_start_sequence = 0;
    d.wave_all_entered_sequence = 0;
    d.wave_cancel_requested = false;
    d.wave_cancel_sequence = 0;
    d.wave_cancel_reason = 0;
    if(d.first_block_reason == 0){
        d.first_block_reason = 0;
        d.first_block_sim_time_us = 0;
        d.first_block_control_sequence = 0;
    }
    d.numerical_guard_triggered = false;
    d.numerical_guard_stage = 0;
    d.safety_guard_triggered = false;
    d.trotting_entered = true;
    d.trotting_enter_sequence = _ctrlComp->fsmSequence;
    d.trotting_exit_sequence = 0;

    ROS_INFO("Trotting entry: inherited body height %.3f m and foot positions; transitioning to %.3f m over %.2f s.",
             _heightTransitionStart, _heightTransitionTarget, _heightTransitionDuration);

    if(amp_obs_thread != nullptr){
        if(amp_obs_thread->joinable()){
            amp_obs_thread->join();
        }
        delete amp_obs_thread;
        amp_obs_thread = nullptr;
    }
    ampthreadRunning = State_Trotting::RUNNING;
    amp_obs_thread = new std::thread(&State_Trotting::save_amp_obs_thread,this);
}

void State_Trotting::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
    _ctrlComp->setAllSwing();
    ampthreadRunning = State_Trotting::STOP;
    if(amp_obs_thread != nullptr){
        if(amp_obs_thread->joinable()){
            amp_obs_thread->join();
        }
        delete amp_obs_thread;
        amp_obs_thread = nullptr;
        std::cout << "amp_obs_thread退出!" << std::endl;
    }
    if (outfile.is_open()) {
        outfile.close();
        std::cout << "文件关闭成功!" << std::endl;
    }
}

void State_Trotting::onControlTimeReset(ControlTimeResetReason){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _pcd = _posBody;
    _heightTransitionStart = _pcd(2);
    _heightTransitionTarget = -_robModel->getFeetPosIdeal()(2, 0);
    _heightTransitionElapsed = 0.0;
    _heightTransitionComplete = false;
    _yawCmd = _lowState->getYaw();
    _Rd = rotz(_yawCmd);
    abortWave("control-time reset", false);
    holdCurrentPose();
}

FSMStateName State_Trotting::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        if (_last_cmd==static_cast<int>(UserCommand::START))
        {
            open_amp_save_file();
            dofPosSwitBeginTime = getTime();
        }
        _last_cmd = static_cast<int>(_lowState->userCmd);
        return FSMStateName::TROTTING;
    }
    else{
        _last_cmd = static_cast<int>(_lowState->userCmd);
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run(){
    _posBody = _est->getPosition();
    _velBody = _est->getVelocity();
    _posFeet2BGlobal = _est->getPosFeet2BGlobal();
    _posFeetGlobal = _est->getFeetPos();
    _velFeetGlobal = _est->getFeetVel();
    _B2G_RotMat = _lowState->getRotMat();
    _G2B_RotMat = _B2G_RotMat.transpose();
    _yaw = _lowState->getYaw();
    _dYaw = _lowState->getDYaw();

    if(!stateEstimateFinite()){
        ROS_ERROR_THROTTLE(1.0, "Trotting received a non-finite state estimate; holding current joint positions.");
        abortWave("non-finite state estimate", true);
        holdCurrentPose();
        return;
    }

    _userValue = _lowState->userValue;

    getUserCmd();
    updateHeightTransition();
    updateWaveReadiness();
    updateRunningWaveSafety();
    if(_waveAbortLatched){
        suppressMotionCommand();
        holdCurrentPose();
        return;
    }
    if(!_waveReady){
        suppressMotionCommand();
    }
    calcCmd();

    if(!commandStateFinite()){
        ROS_ERROR_THROTTLE(1.0, "Trotting command calculation became non-finite; holding current joint positions.");
        abortWave("non-finite command state", true);
        holdCurrentPose();
        return;
    }

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight);
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau();
    calcQQd();

    if(!controlOutputFinite()){
        ROS_ERROR_THROTTLE(
            1.0,
            "Trotting output non-finite (foot_p=%d foot_v=%d force_g=%d force_b=%d q=%d qd=%d tau=%d); blocking motor command.",
            _posFeetGlobalGoal.allFinite(), _velFeetGlobalGoal.allFinite(),
            _forceFeetGlobal.allFinite(), _forceFeetBody.allFinite(),
            _qGoal.allFinite(), _qdGoal.allFinite(), _tau.allFinite());
        abortWave("non-finite control output", true);
        holdCurrentPose();
        return;
    }

    const bool stepRequested = checkStepOrNot();
    if(stepRequested && _waveReady && !_waveAbortLatched){
        if(!_waveStarted){
            ROS_INFO("Trotting wave started after height, velocity, attitude, and four-foot contact checks passed.");
        }
        _ctrlComp->setStartWave();
        _waveStarted = true;
        // ---- G2-D1: record wave start ----
        auto &d = _ctrlComp->preWave;
        if(!d.wave_start_requested){
            d.wave_start_requested = true;
            d.wave_start_sequence = _ctrlComp->waveSequence;
        }
    }else{
        _ctrlComp->setAllStance();
        if(_waveStarted && !stepRequested){
            _waveStarted = false;
            _waveReady = false;
            _readinessStableElapsed = 0.0;
            ROS_INFO("Trotting wave stopped; readiness must be re-established before restart.");
        }
    }

    _lowCmd->setTau(_tau);
    _lowCmd->setQ(vec34ToVec12(_qGoal));
    _lowCmd->setQd(vec34ToVec12(_qdGoal));

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i);
        }
    }

}

bool State_Trotting::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        // FALCO commonly requests 0.18--0.20 rad/s while the navigation
        // bridge is capped at 0.22 rad/s.  A 0.20 deadband made valid pure
        // rotation commands repeatedly stop the gait.  Use the same small
        // command deadband as translational motion.
        (fabs(_dYawCmd) > 0.03) ){
        return true;
    }else{
        return false;
    }
}

void State_Trotting::setHighCmd(double vx, double vy, double wz){
    if(!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)){
        resetCommandState();
        return;
    }
    _vCmdBody(0) = saturation(vx, _vxLim);
    _vCmdBody(1) = saturation(vy, _vyLim);
    _vCmdBody(2) = 0; 
    _dYawCmd = saturation(wz, _wyawLim);
    _dYawCmdPast = _dYawCmd;
}

void State_Trotting::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    const ros::Time callbackControlTime = _ctrlComp->controlTime.isZero() ?
        ros::Time::now() : _ctrlComp->controlTime;
    if(!msg || !std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
       !std::isfinite(msg->angular.z)){
        _cmdVelActive = true;
        _cmdVx = 0.0;
        _cmdVy = 0.0;
        _cmdWz = 0.0;
        _lastCmdVelTime = callbackControlTime;
        ROS_WARN_THROTTLE(1.0, "Trotting rejected a non-finite /cmd_vel and commanded a stop.");
        return;
    }
    _cmdVelActive = true;
    _cmdVx = msg->linear.x;
    _cmdVy = msg->linear.y;
    _cmdWz = msg->angular.z;
    // Use the same accepted Gazebo time as getUserCmd().  Under a heavy DSV
    // planning load, ros::Time::now() can lag the controller's queued state
    // timestamp enough to make a live command appear older than the timeout.
    _lastCmdVelTime = callbackControlTime;
}

void State_Trotting::getUserCmd(){
    const ros::Time now = _ctrlComp->controlTime.isZero() ? ros::Time::now() :
        _ctrlComp->controlTime;
    const double cmdAge = _lastCmdVelTime.isZero() ? -1.0 :
        (now - _lastCmdVelTime).toSec();
    const bool cmdVelFresh = _cmdVelActive && cmdAge >= 0.0 &&
        cmdAge <= _cmdVelTimeout;

    if(_cmdVelActive && !cmdVelFresh){
        _cmdVelActive = false;
        _cmdVx = 0.0;
        _cmdVy = 0.0;
        _cmdWz = 0.0;
        ROS_WARN_THROTTLE(1.0, "Trotting /cmd_vel timed out; commanding zero velocity.");
    }

    if (cmdVelFresh){
        /* Use /cmd_vel when available */
        _vCmdBody(0) = saturation(_cmdVx, _vxLim);
        _vCmdBody(1) = saturation(_cmdVy, _vyLim);
        _vCmdBody(2) = 0;
        _dYawCmd = saturation(_cmdWz, _wyawLim);
        _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
        _dYawCmd = saturation(_dYawCmd, _wyawLim);
        _dYawCmdPast = _dYawCmd;
    } else {
        /* Movement */
        _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
        _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
        _vCmdBody(2) = 0;

        /* Turning */
        _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
        _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd;
        _dYawCmd = saturation(_dYawCmd, _wyawLim);
        _dYawCmdPast = _dYawCmd;
    }
    _ctrlComp->resolvedVx = _vCmdBody(0);
    _ctrlComp->resolvedVy = _vCmdBody(1);
    _ctrlComp->resolvedYawRate = _dYawCmd;
}

bool State_Trotting::stateEstimateFinite() const{
    return _posBody.allFinite() && _velBody.allFinite() &&
           _posFeet2BGlobal.allFinite() && _posFeetGlobal.allFinite() &&
           _velFeetGlobal.allFinite() && _B2G_RotMat.allFinite() &&
           _G2B_RotMat.allFinite() && std::isfinite(_yaw) &&
           std::isfinite(_dYaw);
}

bool State_Trotting::commandStateFinite() const{
    return _pcd.allFinite() && _vCmdBody.allFinite() &&
           _vCmdGlobal.allFinite() && _wCmdGlobal.allFinite() &&
           _Rd.allFinite() && std::isfinite(_yawCmd) &&
           std::isfinite(_dYawCmd) && std::isfinite(_dYawCmdPast);
}

bool State_Trotting::controlOutputFinite() const{
    return _posFeetGlobalGoal.allFinite() && _velFeetGlobalGoal.allFinite() &&
           _forceFeetGlobal.allFinite() && _forceFeetBody.allFinite() &&
           _qGoal.allFinite() && _qdGoal.allFinite() && _tau.allFinite();
}

void State_Trotting::holdCurrentPose(){
    _ctrlComp->setAllStanceNow();
    for(int i=0; i<12; ++i){
        const double measuredQ = _lowState->motorState[i].q;
        _lowCmd->motorCmd[i].q = std::isfinite(measuredQ) ? measuredQ : 0.0;
        _lowCmd->motorCmd[i].dq = 0.0;
        _lowCmd->motorCmd[i].tau = 0.0;
    }
    for(int leg=0; leg<4; ++leg){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
            _lowCmd->setSimStanceGain(leg);
        }else{
            _lowCmd->setRealStanceGain(leg);
        }
    }
}

void State_Trotting::resetCommandState(){
    _vCmdGlobal.setZero();
    _vCmdBody.setZero();
    _wCmdGlobal.setZero();
    _dYawCmd = 0.0;
    _dYawCmdPast = 0.0;
    _ctrlComp->resolvedVx = 0.0;
    _ctrlComp->resolvedVy = 0.0;
    _ctrlComp->resolvedYawRate = 0.0;
    _cmdVelActive = false;
    _cmdVx = 0.0;
    _cmdVy = 0.0;
    _cmdWz = 0.0;
    _lastCmdVelTime = ros::Time();
}

void State_Trotting::updateHeightTransition(){
    if(_heightTransitionComplete){
        _pcd(2) = _heightTransitionTarget;
        return;
    }

    _heightTransitionElapsed += _ctrlComp->getControlDt();
    double ratio = _heightTransitionElapsed / _heightTransitionDuration;
    if(ratio >= 1.0){
        ratio = 1.0;
        _heightTransitionComplete = true;
    }else if(ratio < 0.0){
        ratio = 0.0;
    }
    const double smoothRatio = ratio * ratio * (3.0 - 2.0 * ratio);
    _pcd(2) = _heightTransitionStart +
        (_heightTransitionTarget - _heightTransitionStart) * smoothRatio;
}

bool State_Trotting::expectedAllStance() const{
    for(int i=0; i<4; ++i){
        if((*_contact)(i) != 1){
            return false;
        }
    }
    return true;
}

bool State_Trotting::readinessConditionsMet() const{
    const Vec3 rpy = rotMatToRPY(_B2G_RotMat);
    const double linearSpeed = _velBody.norm();
    const double angularSpeed = _lowState->getGyroGlobal().norm();
    return _heightTransitionComplete && expectedAllStance() &&
           _lowState->hasAllFeetContact(static_cast<float>(_minimumContactForce)) &&
           linearSpeed < _readyLinearVelocity &&
           angularSpeed < _readyAngularVelocity &&
           std::abs(rpy(0)) < _readyTilt && std::abs(rpy(1)) < _readyTilt;
}

void State_Trotting::updateWaveReadiness(){
    if(_waveStarted || _waveAbortLatched){
        return;
    }

    if(_waveReady){
        if(readinessConditionsMet()){
            return;
        }
        _waveReady = false;
        _readinessStableElapsed = 0.0;
        ROS_WARN("Trotting readiness was lost before wave start; returning to all-stance hold.");
    }

    const bool condMet = readinessConditionsMet();
    if(condMet){
        _readinessStableElapsed += _ctrlComp->getControlDt();
    }else{
        _readinessStableElapsed = 0.0;
    }

    if(_readinessStableElapsed >= _readinessHoldDuration){
        _waveReady = true;
        ROS_INFO("Trotting wave ready after %.2f s stable: |v|=%.3f m/s, |w|=%.3f rad/s, force=[%.1f %.1f %.1f %.1f] N.",
                 _readinessStableElapsed, _velBody.norm(),
                 _lowState->getGyroGlobal().norm(),
                 _lowState->footForce[0], _lowState->footForce[1],
                 _lowState->footForce[2], _lowState->footForce[3]);
    }

    // ---- G2-D1: populate readiness diagnostics (write-only) ----
    auto &d = _ctrlComp->preWave;
    d.height_ready = _heightTransitionComplete;
    d.stance_ready = expectedAllStance();
    d.contact_ready = _lowState->hasAllFeetContact(static_cast<float>(_minimumContactForce));
    for(int i=0; i<4; ++i){
        d.contact_fresh[i] = _lowState->footForceValid[i];
        d.contact_force[i] = _lowState->footForce[i];
    }
    const Vec3 rpyDiag = rotMatToRPY(_B2G_RotMat);
    d.linear_speed_ready = _velBody.norm() < _readyLinearVelocity;
    d.angular_speed_ready = _lowState->getGyroGlobal().norm() < _readyAngularVelocity;
    d.tilt_ready = std::abs(rpyDiag(0)) < _readyTilt && std::abs(rpyDiag(1)) < _readyTilt;
    d.readiness_met = condMet;
    d.readiness_hold_complete = _waveReady;
    d.readiness_hold_elapsed = _readinessStableElapsed;
    d.linear_speed = _velBody.norm();
    d.angular_speed = _lowState->getGyroGlobal().norm();
    d.roll_deg = rpyDiag(0) * 180.0 / M_PI;
    d.pitch_deg = rpyDiag(1) * 180.0 / M_PI;
    d.model_height = _posBody(2);

    ROS_INFO_THROTTLE(
        1.0,
        "[TROTTING_READINESS] height=%d stance=%d contact=%d linear=%d angular=%d tilt=%d hold=%.3f ready=%d "
        "speed=%.3f gyro=%.3f rpy=[%.2f %.2f] force=[%.1f %.1f %.1f %.1f]",
        d.height_ready, d.stance_ready, d.contact_ready,
        d.linear_speed_ready, d.angular_speed_ready, d.tilt_ready,
        d.readiness_hold_elapsed, d.readiness_hold_complete,
        d.linear_speed, d.angular_speed, d.roll_deg, d.pitch_deg,
        d.contact_force[0], d.contact_force[1],
        d.contact_force[2], d.contact_force[3]);

    // First block latch: set once on first detected failure condition
    if(d.first_block_reason == 0 && !_waveReady){
        if(!_heightTransitionComplete){
            d.first_block_reason = 101;  // READINESS_HEIGHT_FALSE
        }else if(!expectedAllStance()){
            d.first_block_reason = 102;  // READINESS_STANCE_FALSE
        }else if(!_lowState->hasAllFeetContact(static_cast<float>(_minimumContactForce))){
            d.first_block_reason = 103;  // READINESS_CONTACT_FALSE
        }
        if(d.first_block_reason != 0){
            d.first_block_sim_time_us = _ctrlComp->controlTime.toNSec() / 1000ULL;
            d.first_block_control_sequence = _ctrlComp->acceptedStateSequence;
        }
    }
}

bool State_Trotting::expectedStanceFeetHaveContact() const{
    int expectedStanceFeet = 0;
    for(int i=0; i<4; ++i){
        if((*_contact)(i) != 1){
            continue;
        }
        ++expectedStanceFeet;
        if(!_lowState->footForceValid[i] ||
           !std::isfinite(_lowState->footForce[i]) ||
           _lowState->footForce[i] < _minimumContactForce){
            return false;
        }
    }
    return expectedStanceFeet >= 2;
}

void State_Trotting::updateRunningWaveSafety(){
    if(!_waveStarted || _waveAbortLatched){
        _waveContactLossElapsed = 0.0;
        return;
    }

    const Vec3 rpy = rotMatToRPY(_B2G_RotMat);
    if(std::abs(rpy(0)) >= _waveAbortTilt ||
       std::abs(rpy(1)) >= _waveAbortTilt){
        ROS_ERROR("Trotting wave abort: attitude exceeded limit (roll=%.1fdeg pitch=%.1fdeg limit=%.1fdeg).",
                  rpy(0) * 180.0 / M_PI, rpy(1) * 180.0 / M_PI,
                  _waveAbortTilt * 180.0 / M_PI);
        abortWave("unsafe body attitude", true);
        return;
    }

    if(expectedStanceFeetHaveContact()){
        _waveContactLossElapsed = 0.0;
        return;
    }

    _waveContactLossElapsed += _ctrlComp->getControlDt();
    if(_waveContactLossElapsed >= _waveContactLossDuration){
        ROS_ERROR("Trotting wave abort: expected stance-foot contact was invalid for %.3f simulated seconds; force=[%.1f %.1f %.1f %.1f] N.",
                  _waveContactLossElapsed,
                  _lowState->footForce[0], _lowState->footForce[1],
                  _lowState->footForce[2], _lowState->footForce[3]);
        abortWave("stance-foot contact loss", true);
    }
}

void State_Trotting::abortWave(const char *reason, bool latchAbort){
    const bool wasActive = _waveStarted || _waveReady;
    _ctrlComp->setAllStanceNow();
    _gait->restart();
    _waveStarted = false;
    _waveReady = false;
    _readinessStableElapsed = 0.0;
    _waveContactLossElapsed = 0.0;
    _waveAbortLatched = latchAbort;
    resetCommandState();
    _ctrlComp->ioInter->zeroCmdPanel();

    // ---- G2-D1: record wave cancel ----
    auto &d = _ctrlComp->preWave;
    d.wave_cancel_requested = true;
    d.wave_cancel_sequence = _ctrlComp->waveSequence;
    if(d.wave_cancel_reason == 0){
        if(std::strcmp(reason, "non-finite state estimate") == 0) d.wave_cancel_reason = 1;
        else if(std::strcmp(reason, "non-finite command state") == 0) d.wave_cancel_reason = 2;
        else if(std::strcmp(reason, "non-finite control output") == 0) d.wave_cancel_reason = 3;
        else if(std::strcmp(reason, "unsafe body attitude") == 0) d.wave_cancel_reason = 4;
        else if(std::strcmp(reason, "stance-foot contact loss") == 0) d.wave_cancel_reason = 5;
        else if(std::strcmp(reason, "control-time reset") == 0) d.wave_cancel_reason = 6;
        else d.wave_cancel_reason = 7;  // unknown
    }
    // Latch first block reason for numerical/safety guards
    if(d.first_block_reason == 0){
        if(d.wave_cancel_reason == 1) d.first_block_reason = 201;  // NUMERICAL_GUARD_STATE
        else if(d.wave_cancel_reason == 2) d.first_block_reason = 202;  // NUMERICAL_GUARD_COMMAND
        else if(d.wave_cancel_reason == 3) d.first_block_reason = 203;  // NUMERICAL_GUARD_OUTPUT
        else if(d.wave_cancel_reason == 4) d.first_block_reason = 301;  // SAFETY_GUARD_ATTITUDE
        else if(d.wave_cancel_reason == 5) d.first_block_reason = 302;  // SAFETY_GUARD_CONTACT
        if(d.first_block_reason != 0){
            d.first_block_sim_time_us = _ctrlComp->controlTime.toNSec() / 1000ULL;
            d.first_block_control_sequence = _ctrlComp->acceptedStateSequence;
        }
    }

    if(_posBody.allFinite()){
        _pcd = _posBody;
    }
    if(_posFeetGlobal.allFinite()){
        _posFeetGlobalGoal = _posFeetGlobal;
        _velFeetGlobalGoal.setZero();
    }
    if(_B2G_RotMat.allFinite()){
        _yawCmd = _lowState->getYaw();
        _Rd = rotz(_yawCmd);
    }

    if(wasActive || latchAbort){
        ROS_WARN("Trotting wave cancelled (%s); holding all stance%s.",
                 reason, latchAbort ? " until the state is re-entered" : "");
    }
}

void State_Trotting::suppressMotionCommand(){
    _vCmdBody.setZero();
    _dYawCmd = 0.0;
    _dYawCmdPast = 0.0;
}

void State_Trotting::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody;

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2));
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2));

    const double simDt = _ctrlComp->getControlDt();
    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * simDt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05));
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * simDt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0;

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * simDt;

    _Rd = rotz(_yawCmd);
    _wCmdGlobal(2) = _dYawCmd;
    // std::cout << "_vCmdBody(0): " << _vCmdBody(0) << std::endl;
    // std::cout << "_vCmdBody(1): " << _vCmdBody(1) << std::endl;
    // std::cout << "_dYawCmd: " << _dYawCmd << std::endl;
}

void State_Trotting::calcTau(){
    _posError = _pcd - _posBody;
    _velError = _vCmdGlobal - _velBody;

    _ddPcd = _Kpp * _posError + _Kdp * _velError;
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3));
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));

    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact);

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        }
    }

    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal;
    _q = vec34ToVec12(_lowState->getQ());
    _tau = _robModel->getTau(_q, _forceFeetBody);
}

void State_Trotting::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}


void State_Trotting::save_amp_obs_thread()
{
    while(ampthreadRunning == State_Trotting::RUNNING)
    {
        if (_ctrlComp->ioInter->buttons[4] == 1 && !outfile.is_open()){
            open_amp_save_file();
            dofPosSwitBeginTime = getRosTime();
        }
        if (_ctrlComp->ioInter->buttons[5] == 1 && outfile.is_open()){
            close_amp_save_file();
        }
        long long _start_time = getRosTime();
        if (outfile.is_open() && (getRosTime() - dofPosSwitBeginTime) < _duration)
        {
            std::cout << "save data" << std::endl;
            refresh_amp_obs();
        }
        rosAbsoluteWait(_start_time, (long long)(amp_duration * 1000000));
    }
    ampthreadRunning = State_Trotting::OVER;
}

void State_Trotting::refresh_amp_obs(){
    auto opts = torch::TensorOptions().dtype(torch::kFloat32);
    motion_time = static_cast<float>(getRosTime() - dofPosSwitBeginTime)/1e6;
    // outfile << "motion_time: " << motion_time << std::endl;
    // outfile << "base_w_pos: ";
    for (int i=0; i<3; i++) {
        base_w_pos[i] = _ctrlComp->ioInter->_base_w_pos[i];
        // outfile << base_w_pos[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "base_ori: ";
    for (int i=0; i<4; i++) {
        base_w_orientation[i] = _ctrlComp->ioInter->_base_w_ori[i];
        // outfile << base_w_orientation[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "dof_pos: ";
    for(int i=0; i<12; i++){
        joint_pos[i] = _lowState->motorState[reindex[i]].q;
        // outfile << joint_pos[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "foot_pos: ";
    for (int i=0; i<3; i++){
        foot_pos[0*3+i] = _ctrlComp->ioInter->_FL_foot_pos[i];
        foot_vel[0*3+i] = _ctrlComp->ioInter->_FL_foot_vel[i];
    }
    for (int i=0; i<3; i++){
        foot_pos[1*3+i] = _ctrlComp->ioInter->_FR_foot_pos[i];
        foot_vel[1*3+i] = _ctrlComp->ioInter->_FR_foot_vel[i];
    }
    for (int i=0; i<3; i++){
        foot_pos[2*3+i] = _ctrlComp->ioInter->_RL_foot_pos[i];
        foot_vel[2*3+i] = _ctrlComp->ioInter->_RL_foot_vel[i];
    }
    for (int i=0; i<3; i++){
        foot_pos[3*3+i] = _ctrlComp->ioInter->_RR_foot_pos[i];
        foot_vel[3*3+i] = _ctrlComp->ioInter->_RR_foot_vel[i];
    }
    for (  int  i=0;i<12;i++ )
    {
        // outfile << foot_pos[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "base_w_linear_vel: ";
    for (int i=0; i<3; i++) {
        base_w_linear_vel[i] = _ctrlComp->ioInter->_base_w_linear_vel[i];
    }
    torch::Tensor orientation_tensor = torch::from_blob(base_w_orientation.data(), {int64_t(base_w_orientation.size())}, opts).unsqueeze(0).clone();
    torch::Tensor w_linear_vel_tensor = torch::from_blob(base_w_linear_vel.data(), {int64_t(base_w_linear_vel.size())}, opts).unsqueeze(0).clone();
    torch::Tensor result = quat_rotate_inverse(orientation_tensor, w_linear_vel_tensor).squeeze().clone();
    for (int i = 0; i < 3; ++i) {
        base_linear_vel[i] = result[i].item<float>();
        std::cout << base_linear_vel[i] << " ";
        // outfile << base_linear_vel[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "base_w_angular_vel: ";
    for (int i=0; i<3; i++) {
        base_w_angular_vel[i] = _ctrlComp->ioInter->_base_w_angular_vel[i];
    }
    torch::Tensor w_angular_vel_tensor = torch::from_blob(base_w_angular_vel.data(), {int64_t(base_w_angular_vel.size())}, opts).unsqueeze(0).clone();
    result = quat_rotate_inverse(orientation_tensor, w_angular_vel_tensor).squeeze().clone();
    for (int i = 0; i < 3; ++i) {
        base_angular_vel[i] = result[i].item<float>();
        std::cout << base_angular_vel[i] << " ";
        // outfile << base_angular_vel[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "dof_vel: ";
    for(int i=0; i<12; i++){
        joint_vel[i] = _lowState->motorState[reindex[i]].dq;
        // outfile << joint_vel[i] << " ";
    }
    // outfile << std::endl;

    // outfile << "foot_vel";
    for (  int  i=0;i<12;i++ )
    {
        // outfile << foot_vel[i] << " ";
    }
    // outfile << std::endl;
    // outfile << std::endl;
    // outfile << std::endl;
    motion_data.insert(motion_data.end(), base_w_pos.begin(), base_w_pos.end());
    motion_data.insert(motion_data.end(), base_w_orientation.begin(), base_w_orientation.end());
    motion_data.insert(motion_data.end(), joint_pos.begin(), joint_pos.end());
    motion_data.insert(motion_data.end(), foot_pos.begin(), foot_pos.end());
    motion_data.insert(motion_data.end(), base_linear_vel.begin(), base_linear_vel.end());
    motion_data.insert(motion_data.end(), base_angular_vel.begin(), base_angular_vel.end());
    motion_data.insert(motion_data.end(), joint_vel.begin(), joint_vel.end());
    motion_data.insert(motion_data.end(), foot_vel.begin(), foot_vel.end());
    outfile << "[";
    for (size_t i = 0; i < motion_data.size(); ++i) {
        outfile << std::fixed << std::setprecision(5) << motion_data[i];
        if (i != motion_data.size() - 1) {
            outfile << ", ";
        }
    }
    motion_data.clear();
    outfile << "],";
    outfile << std::endl;
}

void State_Trotting::open_amp_save_file()
{
    const std::string content = R"({
"LoopMode": "Wrap",
"FrameDuration": 0.020,
"EnableCycleOffsetPosition": true,
"EnableCycleOffsetRotation": true,
"MotionWeight": 1,

"Frames":
[
)";
    // 获取当前系统时间
    std::time_t cTime = std::time(nullptr);
    std::tm* currentTm = std::localtime(&cTime);
    // 构建文件名，格式为 systime + 年-月-日.txt
    std::ostringstream fileNameStream;
    fileNameStream << "/home/chy/log/gazebo/" << "temp";
    std::string fileName = fileNameStream.str();
    // 以追加模式打开文件
    outfile = std::ofstream(fileName, std::ios::out | std::ios::app);
    if (!outfile) {
        std::cerr << "无法打开文件!" << std::endl;
    } else {
        std::cout << "文件打开成功!" << std::endl;
        outfile << content;
    }
}

void State_Trotting::close_amp_save_file()
{
    if (outfile.is_open()) {
        // 获取当前文件的写入位置
        std::streampos current_pos = outfile.tellp();

        if (current_pos > 0) {
            // 退回一个字符
            outfile.seekp(current_pos - std::streamoff(1));
        }

        // 写入结尾内容
        const std::string content = R"(]
}
)";
        outfile << content;

        // 关闭文件
        outfile.close();
        std::cout << "文件关闭保存成功!" << std::endl;
    }
}

torch::Tensor State_Trotting::quat_rotate_inverse(const torch::Tensor& q, const torch::Tensor& v) {
    // Ensure q and v are of the correct shape: (batch_size, 4) for quaternions and (batch_size, 3) for vectors
    auto shape = q.sizes();
    // std::cout << "shape: " << shape << std::endl;
    auto q_w = q.index({torch::indexing::Slice(), 3});  // last column is the w component
    // std::cout << "q_w: " << q_w << std::endl;
    auto q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});  // first three columns are the vector part
    // std::cout << "q_vec: " << q_vec << std::endl;
    // a = v * (2.0 * q_w^2 - 1.0).unsqueeze(-1)
    auto a = v * (2.0 * q_w.pow(2) - 1.0).unsqueeze(-1);
    // std::cout << "a: " << a << std::endl;
    // b = cross(q_vec, v) * q_w.unsqueeze(-1) * 2.0
    auto b = torch::cross(q_vec, v, /*dim=*/-1) * q_w.unsqueeze(-1) * 2.0;
    // std::cout << "b: " << b << std::endl;
    // c = q_vec * torch::bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    auto q_vec_reshaped = q_vec.view({shape[0], 1, 3});
    // std::cout << "q_vec_reshaped: " << q_vec_reshaped << std::endl;
    auto v_reshaped = v.view({shape[0], 3, 1});
    // std::cout << "v_reshaped: " << v_reshaped << std::endl;
    auto c = q_vec * torch::bmm(q_vec_reshaped, v_reshaped).squeeze(-1) * 2.0;
    // std::cout << "c: " << c << std::endl;
    // Return a - b + c
    // std::cout << "a - b + c: " << a - b + c << std::endl;
    return a - b + c;
}
