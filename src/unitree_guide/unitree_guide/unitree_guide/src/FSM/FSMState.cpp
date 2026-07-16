/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSMState.h"
#include "common/TimingAlignment.h"

FSMState::FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString)
            :_ctrlComp(ctrlComp), _stateName(stateName), _stateNameString(stateNameString){
    _lowCmd = _ctrlComp->lowCmd;
    _lowState = _ctrlComp->lowState;
}

long long FSMState::getRosTime()
{
    return static_cast<long long>(
        _ctrlComp->ioInter->current_time.load(std::memory_order_acquire));
}

long long  FSMState::getTime()
{
    if (real == false)
    {
        return getRosTime();
    }
    else
    {
        return getSystemTime();
    }
}

PolicyWaitExitReason FSMState::wait(long long startTime, long long waitTime){
    if (real == false)
    {
        return rosAbsoluteWait(startTime,waitTime);
    }
    else
    {
        absoluteWait(startTime,waitTime);
        return ros::ok() ? PolicyWaitExitReason::SimPeriodReached :
                           PolicyWaitExitReason::Shutdown;
    }
}

PolicyWaitExitReason FSMState::rosAbsoluteWait(long long startTime, long long waitTime){
    overtime = 0;
    long long elapsed = getRosTime() - startTime;
    long long now = getSystemTime();
    if(elapsed > waitTime && shouldLogAbsoluteWaitWarning(now)){
        std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        << "The program has already cost " << elapsed << "us." << std::endl;
    }
    while((getRosTime() - startTime < waitTime) && (overtime < OVERTIME)){

        if(classifyPolicyWaitExit(
               static_cast<std::uint64_t>(getRosTime()),
               static_cast<std::uint64_t>(startTime),
               static_cast<std::uint64_t>(waitTime), false) ==
           PolicyWaitExitReason::SimTimeReset){
            return PolicyWaitExitReason::SimTimeReset;
        }

        // std::cout << "getRosTime()" << getRosTime() << std::endl;
        // std::cout << "startTime" << startTime << std::endl;
        // std::cout << "wait" << std::endl;
        // std::cout << "overtime" <<  overtime << std::endl;
        usleep(50);
        overtime += 50;
    }
    return classifyPolicyWaitExit(
        static_cast<std::uint64_t>(getRosTime()),
        static_cast<std::uint64_t>(startTime),
        static_cast<std::uint64_t>(waitTime), !ros::ok());
}

//设置cmd_vel的回调函数，将move_base转化为
void FSMState::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
   if (msg) {
        current_cmd_vel_.linear_x = msg->linear.x;
        current_cmd_vel_.linear_y = msg->linear.y;
        current_cmd_vel_.angular_z = msg->angular.z;
        current_cmd_vel_.valid = true;
        current_cmd_vel_.stamp = ros::Time::now();
    }
    // std::cout << "cmd_vel_linear_x"<< this->current_cmd_vel_.linear_x<< std::endl;
    // std::cout << "cmd_vel_linear_y"<< this->current_cmd_vel_.linear_y<< std::endl;
    // std::cout << "cmd_vel_angular_z"<< this->current_cmd_vel_.angular_z<< std::endl;
}
