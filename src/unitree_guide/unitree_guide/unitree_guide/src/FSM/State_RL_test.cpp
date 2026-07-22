/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include "FSM/State_RL_test.h"

namespace {
const char *kDefaultPolicyPath = "src/unitree_guide/logs/policy_act_inference_stair.pt";

std::string shellQuote(const std::string &value)
{
    std::string quoted = "'";
    for(char c : value){
        if(c == '\''){
            quoted += "'\\''";
        }else{
            quoted += c;
        }
    }
    quoted += "'";
    return quoted;
}

std::string resolvedPath(const std::string &path)
{
    char *resolved = realpath(path.c_str(), nullptr);
    if(resolved == nullptr){
        return "";
    }
    std::string out(resolved);
    std::free(resolved);
    return out;
}

bool fileExists(const std::string &path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

std::string sha256sumForFile(const std::string &path)
{
    if(!fileExists(path)){
        return "";
    }
    const std::string command = "sha256sum " + shellQuote(path);
    FILE *pipe = popen(command.c_str(), "r");
    if(pipe == nullptr){
        return "";
    }
    char buffer[256];
    std::string line;
    if(fgets(buffer, sizeof(buffer), pipe) != nullptr){
        line = buffer;
    }
    pclose(pipe);
    std::istringstream input(line);
    std::string sha;
    input >> sha;
    return sha;
}

std::string configuredPolicyPath()
{
    std::string path = kDefaultPolicyPath;
    std::string rosPath;
    if(ros::param::get("/rl_policy_path", rosPath) && !rosPath.empty()){
        return rosPath;
    }
    const char *envPath = std::getenv("RL_POLICY_PATH");
    if(envPath != nullptr && envPath[0] != '\0'){
        path = envPath;
    }
    return path;
}
}

State_RL::State_RL(CtrlComponents *ctrlComp)
                :FSMState(ctrlComp, FSMStateName::RL, "RL")
{
    load_policy();
    gravity(0,0) = 0.0;
    gravity(1,0) = 0.0;
    gravity(2,0) = -0.98;
    //在构造函数中初始化，订阅
    this->Sub_=nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1000,
        boost::bind(&State_RL::cmdVelCallback,this,_1));

}


void State_RL::enter(){
    // Gazebo low-level command initialization.
        for(int i=0; i<12; i++){
            _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
            _startPos[i] = _lowState->motorState[i].q;
            _lowCmd->motorCmd[i].mode = 10;
            _lowCmd->motorCmd[i].dq = 0;
            _lowCmd->motorCmd[i].Kp = 80;
            _lowCmd->motorCmd[i].Kd = 1;
            _lowCmd->motorCmd[i].tau = 0;
        }
        for(int i=0; i<4; i++){
             if(_ctrlComp->ctrlPlatform == CtrlPlatform::GAZEBO){
                 _lowCmd->setSimStanceGain(i);
             }
             else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
                 _lowCmd->setRealStanceGain(i);
             }
             _lowCmd->setZeroDq(i);
             _lowCmd->setZeroTau(i);
        }
    // Real-robot FreeDog initialization is intentionally disabled here.
    // The old commented-out condition left this block unconditional, causing
    // Gazebo RL entry to call ioInterFreeDog->setCmd() as well.
    /*
    for(int i=0; i<12; i++){
        float c_joint = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[i].q;
        std::vector<double> joint{c_joint, 0, 0, 80, 1};
        _ctrlComp->ioInterFreeDog->setCmd(i,joint);
    }
    */
    obs_history_tensor = torch::zeros({HISTORY_LEN, 45}).to(device);
    actions_tensor = torch::zeros({12});
    actions_tensor_scaled = torch::zeros({12});
    last_actions.fill(0.0f);
    history_stamps_us_.fill(0);
    history_duplicate_count_ = 0;
    history_gate_.reset();
    policy_sequence_ = 0;
    action_sequence_ = 0;
    last_applied_action_sequence_ = 0;
    handled_reset_generation_ = reset_generation_.load(std::memory_order_acquire);
    action_buffer_.invalidate();
    openDeploymentDiagnostics();
    for (int i = 0; i < HISTORY_LEN; i++)
    {
        PolicyInputSnapshot stateSnapshot;
        PolicyCommandSnapshot commandSnapshot;
        _ctrlComp->ioInter->getPolicyInputSnapshot(stateSnapshot);
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            commandSnapshot = command_snapshot_;
        }
        refresh_rl_obs(&stateSnapshot, &commandSnapshot, true);
    }
    infer_thread_runnning.store(State_RL::RUNNING, std::memory_order_release);
    infer_thread = new std::thread(&State_RL::infer_thread_callback,this);
    if (debug == true){
        ampthreadRunning.store(State_RL::RUNNING, std::memory_order_release);
        amp_obs_thread = new std::thread(&State_RL::save_amp_obs_thread,this);
    }
}

void State_RL::onControlTimeReset(ControlTimeResetReason resetReason){
    if(resetReason == ControlTimeResetReason::Paused){
        return;
    }
    action_buffer_.invalidate();
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_snapshot_ = PolicyCommandSnapshot{};
    }
    TimingDiagnostics &diagnostics = TimingDiagnostics::instance();
    diagnostics.beginActionWrite();
    for(int i=0; i<12; ++i){
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
    }
    diagnostics.endActionWrite(0, _ctrlComp->ioInter->stateSequence(),
                               _ctrlComp->ioInter->stateStampUs());
    last_applied_action_sequence_ = 0;
    dofPosSwitBeginTime = getTime();
    _percent = 0;
    reset_generation_.fetch_add(1, std::memory_order_acq_rel);
}

void State_RL::resetPolicyStateForTimeDiscontinuity(){
    obs_history_tensor = torch::zeros({HISTORY_LEN, 45}).to(device);
    actions_tensor = torch::zeros({12});
    actions_tensor_scaled = torch::zeros({12});
    last_actions.fill(0.0f);
    history_stamps_us_.fill(0);
    history_duplicate_count_ = 0;
    history_gate_.reset();
    policy_sequence_ = 0;
    action_sequence_ = 0;

    PolicyInputSnapshot stateSnapshot;
    PolicyCommandSnapshot commandSnapshot;
    if(!_ctrlComp->ioInter->getPolicyInputSnapshot(stateSnapshot)){
        return;
    }
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        commandSnapshot = command_snapshot_;
    }
    for(int i=0; i<HISTORY_LEN; ++i){
        refresh_rl_obs(&stateSnapshot, &commandSnapshot, true);
    }
}

void State_RL::run(){
    const PolicyOutputSnapshot snapshot = action_buffer_.read();
    if(!snapshot.valid ||
       snapshot.reset_generation != reset_generation_.load(std::memory_order_acquire) ||
       snapshot.action_sequence == last_applied_action_sequence_){
        return;
    }
    TimingDiagnostics &diagnostics = TimingDiagnostics::instance();
    diagnostics.beginActionWrite();
    if(real == false){
        for(int i=0; i<12; ++i){
            _lowCmd->motorCmd[i].q = snapshot.q_target[i];
            _lowCmd->motorCmd[i].Kp = 80;
            _lowCmd->motorCmd[i].Kd = 1;
        }
    }
    diagnostics.endActionWrite(snapshot.action_sequence,
                               snapshot.source_state_sequence,
                               snapshot.source_sim_time_us);
    last_applied_action_sequence_ = snapshot.action_sequence;
}

void State_RL::exit(){
    _percent = 0;
    ampthreadRunning.store(State_RL::STOP, std::memory_order_release);
    infer_thread_runnning.store(State_RL::STOP, std::memory_order_release);
    if(amp_obs_thread != nullptr){
        if(amp_obs_thread->joinable()){
            amp_obs_thread->join();
        }
        delete amp_obs_thread;
        amp_obs_thread = nullptr;
        std::cout << "amp_obs_thread退出!" << std::endl;
    }
    if(infer_thread != nullptr){
        if(infer_thread->joinable()){
            infer_thread->join();
        }
        delete infer_thread;
        infer_thread = nullptr;
        std::cout << "infer_thread退出!" << std::endl;
    }
    if (outfile.is_open()) {
        outfile.close();
        std::cout << "文件关闭成功!" << std::endl;
    }
    closeDeploymentDiagnostics();
}

FSMStateName State_RL::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::L1_X){
        if (_last_cmd==static_cast<int>(UserCommand::RL))
        {
            _cnt = (_cnt+1)%(sizeof(_targetPos_map) / sizeof(_targetPos_map[0]));
            if (real == false){
                for(int i=0; i<12; i++){
                    _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
                    _startPos[i] = _lowState->motorState[i].q;
                }
            }
            else if(real == true){
                for(int i=0; i<12; i++){
                    _startPos[i] = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[i].q;
                }
            }
            _percent = 0;
            std::cout << "cnt: " << _cnt << std::endl;
            // open_amp_save_file();
            dofPosSwitBeginTime = getTime();
        }
        _last_cmd = static_cast<int>(_lowState->userCmd);
        return FSMStateName::RL;
    }
    else{
        _last_cmd = static_cast<int>(_lowState->userCmd);
        return FSMStateName::RL;
    }
}

void State_RL::infer_thread_callback()
{
    while(infer_thread_runnning.load(std::memory_order_acquire) == State_RL::RUNNING)
    {
        const std::uint64_t resetGeneration =
            reset_generation_.load(std::memory_order_acquire);
        if(resetGeneration != handled_reset_generation_){
            resetPolicyStateForTimeDiscontinuity();
            handled_reset_generation_ = resetGeneration;
        }
        long long _start_time = getTime();
        const std::uint64_t wallStartNs = ros::WallTime::now().toNSec();
        PolicyInputSnapshot stateSnapshot;
        PolicyCommandSnapshot commandSnapshot;
        if(!_ctrlComp->ioInter->getPolicyInputSnapshot(stateSnapshot)){
            wait(_start_time, (long long)(infer_duration * 1000000));
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            commandSnapshot = command_snapshot_;
        }
        const std::uint64_t sourceStateSequence = stateSnapshot.state_sequence;
        const std::uint64_t sourceSimTimeUs = stateSnapshot.sim_time_us;
        // std::cout << "_start_time" << _start_time << std::endl;
        if(!refresh_rl_obs(&stateSnapshot, &commandSnapshot)){
            usleep(50);
            continue;
        }
        const std::uint64_t policySequence = ++policy_sequence_;
        torch::Tensor flattened_obs = obs_history_tensor.view({1, HISTORY_LEN * 45});
        if (debug == true)
        {
            const std::vector<int> sub_sizes = {3, 3, 3, 12, 12, 12};
            int segment_size = 45;
            // std::cout << "printSegments" << std::endl;
            // printSegments(flattened_obs.squeeze(), segment_size, sub_sizes);
        }
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(flattened_obs);
        // std::cout << "flattened_obs: " << flattened_obs << std::endl;
        actions_tensor = model.get_method("act_inference")(inputs).toTensor().to(torch::kCPU).squeeze();
        if (debug==true){
            torch::Tensor input_tensor = torch::arange(1, 226).view({1, 225}).to(torch::kFloat32).to(device); // 注意范围是 [start, end)
            std::vector<torch::jit::IValue> test;
            test.push_back(input_tensor);
            torch::Tensor output = model.get_method("act_inference")(test).toTensor().to(torch::kCPU).squeeze();
            // printTensorHorizontal(output, "same_net_work_test");
        }
        actions_tensor_scaled = actions_tensor.clone() * 0.25;
        std::vector<float> actions(actions_tensor_scaled.data_ptr<float>(),
                           actions_tensor_scaled.data_ptr<float>() + actions_tensor_scaled.numel());
        if (debug == true) std::cout << "actions[reindex[j]]  + default_dof_pos" << std::endl;
        PolicyOutputSnapshot outputSnapshot;
        outputSnapshot.action_sequence = ++action_sequence_;
        outputSnapshot.source_state_sequence = sourceStateSequence;
        outputSnapshot.source_sim_time_us = sourceSimTimeUs;
        outputSnapshot.reset_generation = resetGeneration;
        outputSnapshot.valid = true;
        for(int i=0; i<12; i++){
            if (real == false)
            {
                outputSnapshot.raw_action[i] = actions[reindex[i]];
                outputSnapshot.q_target[i] = actions[reindex[i]] +
                    default_dof_pos_tensor[reindex[i]].item<float>();
            }
            else if (real == true)
            {
                // float t_joint = actions[reindex[i]]  + default_dof_pos_tensor[reindex[i]].item<float>();
                // std::vector<double> joint{t_joint, 0, 0, 80, 1};
                // _ctrlComp->ioInterFreeDog->setCmd(i,joint);
            }
            if (debug == true) std::cout << actions[reindex[i]]  + default_dof_pos_tensor[reindex[i]].item<float>() << " ";
        }
        recordDeploymentDiagnostics(policySequence, sourceStateSequence,
                                    sourceSimTimeUs, commandSnapshot,
                                    outputSnapshot);
        action_buffer_.publish(outputSnapshot);
        const std::uint64_t actionSequence = outputSnapshot.action_sequence;
        TimingDiagnostics &diagnostics = TimingDiagnostics::instance();
        TimingRecord actionTiming;
        actionTiming.event = "ACTION";
        actionTiming.wall_time_ns = ros::WallTime::now().toNSec();
        actionTiming.sim_time_us = _ctrlComp->ioInter->stateStampUs();
        actionTiming.state_sequence = _ctrlComp->ioInter->stateSequence();
        actionTiming.state_stamp_us = _ctrlComp->ioInter->stateStampUs();
        actionTiming.policy_sequence = policySequence;
        actionTiming.policy_source_state_sequence = sourceStateSequence;
        actionTiming.policy_sim_time_us = sourceSimTimeUs;
        actionTiming.policy_wall_time_ns = wallStartNs;
        actionTiming.action_sequence = actionSequence;
        actionTiming.action_source_state_sequence = sourceStateSequence;
        diagnostics.record(actionTiming);
        if (debug == true)
            std::cout << std::endl;
        // std::cout << "actions_tensor: " << actions_tensor << std::endl;
        PolicyWaitExitReason waitResult = PolicyWaitExitReason::WallOvertime;
        do {
            const std::uint64_t waitWallStartNs = ros::WallTime::now().toNSec();
            waitResult = wait(_start_time, (long long)(infer_duration * 1000000));
            const std::int64_t simElapsed = getTime() - _start_time;
            TimingRecord waitTiming;
            waitTiming.event = "POLICY_WAIT";
            waitTiming.wall_time_ns = ros::WallTime::now().toNSec();
            waitTiming.sim_time_us = _ctrlComp->ioInter->stateStampUs();
            waitTiming.state_sequence = _ctrlComp->ioInter->stateSequence();
            waitTiming.state_stamp_us = _ctrlComp->ioInter->stateStampUs();
            waitTiming.policy_sequence = policySequence;
            waitTiming.policy_source_state_sequence = sourceStateSequence;
            waitTiming.policy_sim_time_us = sourceSimTimeUs;
            waitTiming.policy_wall_time_ns = wallStartNs;
            waitTiming.policy_wait_exit_reason = policyWaitExitReasonName(waitResult);
            waitTiming.policy_wait_sim_elapsed_us = simElapsed;
            waitTiming.policy_wait_wall_elapsed_us =
                (waitTiming.wall_time_ns - waitWallStartNs) / 1000U;
            waitTiming.history_oldest_stamp_us = history_stamps_us_.front();
            waitTiming.history_newest_stamp_us = history_stamps_us_.back();
            waitTiming.history_span_us = history_stamps_us_.back() >= history_stamps_us_.front() ?
                history_stamps_us_.back() - history_stamps_us_.front() : 0;
            waitTiming.history_duplicate_count = history_duplicate_count_;
            waitTiming.action_sequence = actionSequence;
            waitTiming.action_source_state_sequence = sourceStateSequence;
            diagnostics.record(waitTiming);

        } while(infer_thread_runnning.load(std::memory_order_acquire) == State_RL::RUNNING &&
                waitResult != PolicyWaitExitReason::SimPeriodReached &&
                waitResult != PolicyWaitExitReason::SimTimeReset &&
                waitResult != PolicyWaitExitReason::Shutdown);

        if(waitResult == PolicyWaitExitReason::Shutdown){
            break;
        }
        if(waitResult == PolicyWaitExitReason::SimTimeReset){
            continue;
        }
    }
    infer_thread_runnning.store(State_RL::OVER, std::memory_order_release);
}

void State_RL::save_amp_obs_thread()
{
    while(ampthreadRunning.load(std::memory_order_acquire) == State_RL::RUNNING)
    {
        long long _start_time = getTime();
        if ((getTime() - dofPosSwitBeginTime)<_duration) {
            _percent = (float)(getTime() - dofPosSwitBeginTime)/_duration;
            _percent = _percent > 1 ? 1 : _percent;
            std::cout << "_percent" << _percent << std::endl;
            // if (real == false){
                std::cout << "_lowCmd->motorCmd ";
                for(int j=0; j<12; j++){
                    std::cout << _targetPos_map[_cnt][reindex[j]] << " ";
                    _lowCmd->motorCmd[j].q = (1 - _percent)*_startPos[j] + _percent*_targetPos_map[_cnt][reindex[j]];
                }
                std::cout << _lowCmd->motorCmd << std::endl;
                std::cout << std::endl;
            // }
            // else if (real == true){
                std::cout << "target_joint";
                for(int j=0; j<12; j++){
                    std::cout << _targetPos_map[_cnt][j] << " ";
                    float t_joint = (1 - _percent)*_startPos[j] + _percent*_targetPos_map[_cnt][reindex[j]];
                    std::vector<double> joint{t_joint, 0, 0, 80, 1};
                    _ctrlComp->ioInterFreeDog->setCmd(j,joint);
                }
                std::cout << std::endl;
            // }
            if ((float)(getTime() - dofPosSwitBeginTime)>(float)_duration*0.95)
                close_amp_save_file();
        }
        if (outfile.is_open())
        {
            std::cout << "save data" << std::endl;
            refresh_amp_obs();
        }
        wait(_start_time, (long long)(infer_duration * 1000000));
    }
    ampthreadRunning.store(State_RL::OVER, std::memory_order_release);
}



bool State_RL::refresh_rl_obs(const PolicyInputSnapshot *stateSnapshot,
                              const PolicyCommandSnapshot *commandSnapshot,
                              bool initializeHistory){
    auto opts = torch::TensorOptions().dtype(torch::kFloat32);
    //gazebo simulation mode
    if (real == false)
    {
        if(stateSnapshot == nullptr || commandSnapshot == nullptr ||
           !stateSnapshot->valid){
            return false;
        }
        if(!initializeHistory && !history_gate_.shouldAppend(
               stateSnapshot->state_sequence, stateSnapshot->sim_time_us)){
            return false;
        }
        using_imu_policy_input_ = !stateSnapshot->base_world_valid;
        if(using_imu_policy_input_){
            base_w_orientation[0] = stateSnapshot->low_state.imu.quaternion[1];
            base_w_orientation[1] = stateSnapshot->low_state.imu.quaternion[2];
            base_w_orientation[2] = stateSnapshot->low_state.imu.quaternion[3];
            base_w_orientation[3] = stateSnapshot->low_state.imu.quaternion[0];
            for (int i=0; i<3; i++) {
                base_w_angular_vel[i] = stateSnapshot->low_state.imu.gyroscope[i];
                base_ang_vel_tensor[i] = base_w_angular_vel[i];
            }
        } else {
            for (int i=0; i<4; i++) {
                base_w_orientation[i] = stateSnapshot->base_w_orientation[i];
            }
            for (int i=0; i<3; i++) {
                base_w_angular_vel[i] = stateSnapshot->base_w_angular_velocity[i];
            }
            torch::Tensor w_angular_vel_tensor = torch::from_blob(base_w_angular_vel.data(), {int64_t(base_w_angular_vel.size())}, opts).unsqueeze(0).clone();
            torch::Tensor orientation_tensor = torch::from_blob(base_w_orientation.data(), {int64_t(base_w_orientation.size())}, opts).unsqueeze(0).clone();
            base_ang_vel_tensor = quat_rotate_inverse(orientation_tensor, w_angular_vel_tensor).squeeze().clone();
        }
        torch::Tensor orientation_tensor = torch::from_blob(base_w_orientation.data(), {int64_t(base_w_orientation.size())}, opts).unsqueeze(0).clone();
        projected_gravity_tensor = quat_rotate_inverse(orientation_tensor, gravity_tensor.unsqueeze(0)).squeeze().clone();
        
        //订阅cmd_vel
        // this->Sub_=nh.subscribe<geometry_msgs::Twist>("/cmd_vel",1000,boost::bind(&FSMState::cmdVelCallback,this,_1));

        // commands_tensor[0] = _ctrlComp->ioInter->axes[1];
        // commands_tensor[1] = _ctrlComp->ioInter->axes[0];
        // commands_tensor[2] = _ctrlComp->ioInter->axes[3]*3.14;

        commands_tensor[0] = commandSnapshot->linear_x;
        commands_tensor[1] = commandSnapshot->linear_y;
        commands_tensor[2] = commandSnapshot->angular_z;


        // std::cout << _ctrlComp->ioInter->axes << std::endl;
        // std::cout << "commands_tensor: " << commands_tensor << std::endl;
        for(int i=0; i<12; i++){
            joint_pos[i] = stateSnapshot->low_state.motorState[reindex[i]].q;
        }
        dof_pos_tensor = torch::from_blob(joint_pos.data(), {int64_t(joint_pos.size())}, opts).clone();
        // printTensorHorizontal(dof_pos_tensor,"dof_pos_tensor");
        for(int i=0; i<12; i++){
            joint_vel[i] = stateSnapshot->low_state.motorState[reindex[i]].dq;
        }
        dof_vel_tensor = torch::from_blob(joint_vel.data(), {int64_t(joint_vel.size())}, opts).clone();
        obs_tensor = torch::cat({
            base_ang_vel_tensor * obs_scales_ang_vel,
            projected_gravity_tensor,
            commands_tensor * commands_scale,
            (dof_pos_tensor - default_dof_pos_tensor) * obs_scales_dof_pos,
            dof_vel_tensor * obs_scales_dof_vel,
            actions_tensor
        }, -1).to(device);
        obs_history_tensor = torch::cat({
            obs_history_tensor.slice(0, 1, HISTORY_LEN).to(device),  // 删除最早的一步
            obs_tensor.unsqueeze(0)  // 将当前 obs_tensor 插入到历史中
        }, 0);  // 按行（第0维）拼接
        const std::uint64_t stampUs = stateSnapshot->sim_time_us;
        if(history_stamps_us_.back() == stampUs){
            ++history_duplicate_count_;
        }
        for(std::size_t i=1; i<history_stamps_us_.size(); ++i){
            history_stamps_us_[i - 1] = history_stamps_us_[i];
        }
        history_stamps_us_.back() = stampUs;
        history_gate_.commit(stateSnapshot->state_sequence, stampUs);
    }
    else if (real == true)
    {
        _B2G_RotMat = _ctrlComp->ioInterFreeDog->getRotMat();
        _G2B_RotMat = _B2G_RotMat.transpose();
        Vec3 projected_gravity = _G2B_RotMat*gravity;
        projected_gravity_tensor = torch::tensor({projected_gravity(0,0), projected_gravity(1,0), projected_gravity(2,0)});
         for (int i=0; i<3; i++) {
            base_ang_vel_tensor[i] = _ctrlComp->ioInterFreeDog->low_state.imu_gyroscope[i];
        }
        commands_tensor[0] = _ctrlComp->ioInter->axes[1];
        commands_tensor[1] = _ctrlComp->ioInter->axes[0];
        commands_tensor[2] = _ctrlComp->ioInter->axes[3]*3.14;
        for(int i=0; i<12; i++){
            joint_pos[i] = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[reindex[i]].q;
        }
        dof_pos_tensor = torch::from_blob(joint_pos.data(), {int64_t(joint_pos.size())}, opts).clone();
        for(int i=0; i<12; i++){
            joint_vel[i] = _ctrlComp->ioInterFreeDog->low_state.motorState_free_dog[reindex[i]].dq;
        }
        dof_vel_tensor = torch::from_blob(joint_vel.data(), {int64_t(joint_vel.size())}, opts).clone();
        obs_tensor = torch::cat({
            base_ang_vel_tensor * obs_scales_ang_vel,
            projected_gravity_tensor,
            commands_tensor * commands_scale,
            (dof_pos_tensor - default_dof_pos_tensor) * obs_scales_dof_pos,
            dof_vel_tensor * obs_scales_dof_vel,
            actions_tensor
        }, -1).to(device);
        obs_history_tensor = torch::cat({
            obs_history_tensor.slice(0, 1, HISTORY_LEN).to(device),  // 删除最早的一步
            obs_tensor.unsqueeze(0)  // 将当前 obs_tensor 插入到历史中
        }, 0);  // 按行（第0维）拼接
    }
    return true;
}

void State_RL::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    if(!msg){
        return;
    }
    PolicyCommandSnapshot snapshot;
    snapshot.linear_x = msg->linear.x;
    snapshot.linear_y = msg->linear.y;
    snapshot.angular_z = msg->angular.z;
    snapshot.stamp = ros::Time::now();
    snapshot.valid = true;
    std::lock_guard<std::mutex> lock(command_mutex_);
    snapshot.sequence = command_snapshot_.sequence + 1;
    command_snapshot_ = snapshot;
}


void State_RL::refresh_rl_obs_real_robot(){

}

void State_RL::refresh_amp_obs(){
    auto opts = torch::TensorOptions().dtype(torch::kFloat32);
    motion_time = static_cast<float>(getRosTime() - dofPosSwitBeginTime)/1e6;
    outfile << "motion_time: " << motion_time << std::endl;
    outfile << "base_w_pos: ";
    for (int i=0; i<3; i++) {
        base_w_pos[i] = _ctrlComp->ioInter->_base_w_pos[i];
        outfile << base_w_pos[i] << " ";
    }
    outfile << std::endl;

    outfile << "base_ori: ";
    for (int i=0; i<4; i++) {
        base_w_orientation[i] = _ctrlComp->ioInter->_base_w_ori[i];
        outfile << base_w_orientation[i] << " ";
    }
    outfile << std::endl;

    outfile << "dof_pos: ";
    for(int i=0; i<12; i++){
        joint_pos[i] = _lowState->motorState[reindex[i]].q;
        outfile << joint_pos[i] << " ";
    }
    outfile << std::endl;

    outfile << "foot_pos: ";
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
        outfile << foot_pos[i] << " ";
    }
    outfile << std::endl;

    outfile << "base_w_linear_vel: ";
    for (int i=0; i<3; i++) {
        base_w_linear_vel[i] = _ctrlComp->ioInter->_base_w_linear_vel[i];
    }
    torch::Tensor orientation_tensor = torch::from_blob(base_w_orientation.data(), {int64_t(base_w_orientation.size())}, opts).unsqueeze(0).clone();
    torch::Tensor w_linear_vel_tensor = torch::from_blob(base_w_linear_vel.data(), {int64_t(base_w_linear_vel.size())}, opts).unsqueeze(0).clone();
    torch::Tensor result = quat_rotate_inverse(orientation_tensor, w_linear_vel_tensor).squeeze().clone();
    for (int i = 0; i < 3; ++i) {
        base_linear_vel[i] = result[i].item<float>();
        // std::cout << base_linear_vel[i] << " ";
        outfile << base_linear_vel[i] << " ";
    }
    outfile << std::endl;

    outfile << "base_w_angular_vel: ";
    for (int i=0; i<3; i++) {
        base_w_angular_vel[i] = _ctrlComp->ioInter->_base_w_angular_vel[i];
    }
    torch::Tensor w_angular_vel_tensor = torch::from_blob(base_w_angular_vel.data(), {int64_t(base_w_angular_vel.size())}, opts).unsqueeze(0).clone();
    result = quat_rotate_inverse(orientation_tensor, w_angular_vel_tensor).squeeze().clone();
    for (int i = 0; i < 3; ++i) {
        base_angular_vel[i] = result[i].item<float>();
        // std::cout << base_angular_vel[i] << " ";
        outfile << base_angular_vel[i] << " ";
    }
    outfile << std::endl;

    outfile << "dof_vel: ";
    for(int i=0; i<12; i++){
        joint_vel[i] = _lowState->motorState[reindex[i]].dq;
        outfile << joint_vel[i] << " ";
    }
    outfile << std::endl;

    outfile << "foot_vel";
    for (  int  i=0;i<12;i++ )
    {
        outfile << foot_vel[i] << " ";
    }
    outfile << std::endl;
    outfile << std::endl;
    outfile << std::endl;
}

void State_RL::open_amp_save_file()
{
    // 打开文件输出流
    // 获取当前系统时间
    std::time_t cTime = std::time(nullptr);
    std::tm* currentTm = std::localtime(&cTime);
    // 构建文件名，格式为 systime + 年-月-日.txt
    std::ostringstream fileNameStream;
    fileNameStream << "/home/chy/log/gazebo/" << angle_names[_cnt];
    std::string fileName = fileNameStream.str();
    // 以追加模式打开文件
    outfile = std::ofstream(fileName, std::ios::out | std::ios::app);
    if (!outfile) {
        std::cerr << "无法打开文件!" << std::endl;
    } else {
        // std::cout << "文件打开成功!" << std::endl;
    }
}

void State_RL::close_amp_save_file()
{
    if (outfile.is_open()) {
        outfile.close();
        // std::cout << "文件关闭保存成功!" << std::endl;
    }
}

void State_RL::openDeploymentDiagnostics()
{
    closeDeploymentDiagnostics();
    const char *path = std::getenv("UNITREE_RL_DIAG_PATH");
    if(path == nullptr || path[0] == '\0'){
        deployment_diag_enabled_ = false;
        return;
    }
    deployment_diag_.open(path, std::ios::out | std::ios::trunc);
    if(!deployment_diag_.is_open()){
        std::cerr << "[RL-DIAG] Unable to open " << path << std::endl;
        deployment_diag_enabled_ = false;
        return;
    }
    deployment_diag_enabled_ = true;
    deployment_diag_ << "policy_sequence,source_state_sequence,source_sim_time_us,"
                     << "using_imu_policy_input,"
                     << "cmd_vx,cmd_vy,cmd_yaw,history_oldest_stamp_us,"
                     << "history_newest_stamp_us,history_span_us,history_duplicate_count";
    for(int i=0; i<3; ++i){ deployment_diag_ << ",base_ang_vel_" << i; }
    for(int i=0; i<3; ++i){ deployment_diag_ << ",projected_gravity_" << i; }
    for(int i=0; i<3; ++i){ deployment_diag_ << ",commands_scaled_" << i; }
    for(int i=0; i<12; ++i){ deployment_diag_ << ",joint_pos_policy_" << i; }
    for(int i=0; i<12; ++i){ deployment_diag_ << ",joint_vel_policy_" << i; }
    for(int i=0; i<12; ++i){ deployment_diag_ << ",prev_action_obs_" << i; }
    for(int i=0; i<12; ++i){ deployment_diag_ << ",raw_action_model_" << i; }
    for(int i=0; i<12; ++i){ deployment_diag_ << ",scaled_action_policy_" << i; }
    for(int i=0; i<12; ++i){ deployment_diag_ << ",q_target_gazebo_" << i; }
    for(int i=0; i<12; ++i){ deployment_diag_ << ",default_policy_" << i; }
    for(int i=0; i<12; ++i){ deployment_diag_ << ",reindex_" << i; }
    deployment_diag_ << '\n';
}

void State_RL::closeDeploymentDiagnostics()
{
    if(deployment_diag_.is_open()){
        deployment_diag_.flush();
        deployment_diag_.close();
    }
    deployment_diag_enabled_ = false;
}

void State_RL::recordDeploymentDiagnostics(std::uint64_t policySequence,
                                           std::uint64_t sourceStateSequence,
                                           std::uint64_t sourceSimTimeUs,
                                           const PolicyCommandSnapshot &commandSnapshot,
                                           const PolicyOutputSnapshot &outputSnapshot)
{
    if(!deployment_diag_enabled_ || !deployment_diag_.is_open()){
        return;
    }
    torch::Tensor rawActions = actions_tensor.to(torch::kCPU).contiguous();
    torch::Tensor scaledActions = actions_tensor_scaled.to(torch::kCPU).contiguous();
    torch::Tensor defaultPos = default_dof_pos_tensor.to(torch::kCPU).contiguous();
    torch::Tensor obsCpu = obs_tensor.to(torch::kCPU).contiguous();
    deployment_diag_ << policySequence << ',' << sourceStateSequence << ','
                     << sourceSimTimeUs << ',' << (using_imu_policy_input_ ? 1 : 0) << ','
                     << commandSnapshot.linear_x << ','
                     << commandSnapshot.linear_y << ',' << commandSnapshot.angular_z << ','
                     << history_stamps_us_.front() << ',' << history_stamps_us_.back() << ','
                     << (history_stamps_us_.back() >= history_stamps_us_.front() ?
                         history_stamps_us_.back() - history_stamps_us_.front() : 0) << ','
                     << history_duplicate_count_;
    deployment_diag_ << std::setprecision(9);
    for(int i=0; i<3; ++i){ deployment_diag_ << ',' << base_ang_vel_tensor[i].item<float>(); }
    for(int i=0; i<3; ++i){ deployment_diag_ << ',' << projected_gravity_tensor[i].item<float>(); }
    for(int i=0; i<3; ++i){ deployment_diag_ << ',' << (commands_tensor[i] * commands_scale[i]).item<float>(); }
    for(int i=0; i<12; ++i){ deployment_diag_ << ',' << dof_pos_tensor[i].item<float>(); }
    for(int i=0; i<12; ++i){ deployment_diag_ << ',' << dof_vel_tensor[i].item<float>(); }
    for(int i=0; i<12; ++i){ deployment_diag_ << ',' << obsCpu[33 + i].item<float>(); }
    for(int i=0; i<12; ++i){ deployment_diag_ << ',' << rawActions[i].item<float>(); }
    for(int i=0; i<12; ++i){ deployment_diag_ << ',' << scaledActions[i].item<float>(); }
    for(int i=0; i<12; ++i){ deployment_diag_ << ',' << outputSnapshot.q_target[i]; }
    for(int i=0; i<12; ++i){ deployment_diag_ << ',' << defaultPos[i].item<float>(); }
    for(int i=0; i<12; ++i){ deployment_diag_ << ',' << reindex[i]; }
    deployment_diag_ << '\n';
}

torch::Tensor State_RL::quat_rotate_inverse(const torch::Tensor& q, const torch::Tensor& v) {
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

void State_RL::load_policy()
{
    model_path = configuredPolicyPath();
    const std::string realPath = resolvedPath(model_path);
    const bool exists = fileExists(model_path);
    const std::string sha256 = sha256sumForFile(model_path);
    std::cout << "[RL-POLICY] configured policy path: " << model_path << std::endl;
    std::cout << "[RL-POLICY] resolved realpath: "
              << (realPath.empty() ? "UNRESOLVED" : realPath) << std::endl;
    std::cout << "[RL-POLICY] SHA256: "
              << (sha256.empty() ? "UNAVAILABLE" : sha256) << std::endl;
    std::cout << "[RL-POLICY] file exists: " << (exists ? "true" : "false") << std::endl;
    // load model from check point
    std::cout << "cuda::is_available():" << torch::cuda::is_available() << std::endl;
    device= torch::kCPU;
    if (torch::cuda::is_available()){
        device = torch::kCUDA;
    }
    model = torch::jit::load(model_path);
    std::cout << "[RL-POLICY] load success: true" << std::endl;
    model.to(device);
    std::cout << "load model to device!" << std::endl;
    model.eval();
}

void State_RL::printSegments(const torch::Tensor& tensor, int segment_size, const std::vector<int>& sub_sizes) {
    int num_segments = tensor.size(0) / segment_size;
    std::cout << "num_segments" << num_segments << tensor.size(0) << segment_size << std::endl;
    for (int seg = 0; seg < num_segments; ++seg) {
        auto segment = tensor.slice(0, seg * segment_size, (seg + 1) * segment_size);
        std::cout << "Segment " << seg + 1 << ":\n";

        int start = 0;
        for (size_t i = 0; i < sub_sizes.size(); ++i) {
            int size = sub_sizes[i];
            auto sub_segment = segment.slice(0, start, start + size);  // 按列（第1维）分割
            std::cout << "  Sub-segment " << i + 1 << " (" << size << " elements): ";
            std::string output_str = "  Sub-segment " + std::to_string(i + 1);
            printTensorHorizontal(sub_segment, output_str);
            start += size;
        }
    }
}

// 横排打印函数
void State_RL::printTensorHorizontal(const torch::Tensor& tensor, const std::string& name) {
    std::cout << name << " (" << tensor.sizes() << "): [ ";
    auto tensor_cpu = tensor.to(torch::kCPU);  // 确保张量在 CPU 上
    auto accessor = tensor_cpu.accessor<float, 1>();  // 假设是一维张量

    for (int i = 0; i < tensor.size(0); ++i) {
        std::cout << accessor[i] << " ";
    }
    std::cout << "]\n";
}
