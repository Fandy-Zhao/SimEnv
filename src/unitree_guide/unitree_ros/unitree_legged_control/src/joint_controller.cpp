/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// #include "unitree_legged_control/joint_controller.h"
#include "joint_controller.h"
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <boost/bind/bind.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>
#include <ros/param.h>
#include <ros/spinner.h>
#include <ros/subscribe_options.h>

// #define rqtTune // use rqt or not

namespace unitree_legged_control 
{
    namespace
    {
        std::mutex diagnosticsMutex;
        std::ofstream diagnosticsStream;
        std::vector<std::string> diagnosticsBuffer;
        std::uint64_t diagnosticsLastFlushWallNs = 0;
        constexpr std::size_t kDiagnosticsFlushRows = 4096;
        constexpr std::uint64_t kDiagnosticsFlushIntervalNs = 1000000000ULL;
        ros::CallbackQueue commandCallbackQueue;
        std::unique_ptr<ros::AsyncSpinner> commandCallbackSpinner;
        std::once_flag commandCallbackSpinnerOnce;

        struct DiagnosticsFlushOnExit
        {
            ~DiagnosticsFlushOnExit()
            {
                std::lock_guard<std::mutex> lock(diagnosticsMutex);
                if(diagnosticsStream.is_open()){
                    for(const auto &line : diagnosticsBuffer){
                        diagnosticsStream << line << '\n';
                    }
                    diagnosticsBuffer.clear();
                    diagnosticsStream.flush();
                }
            }
        };

        DiagnosticsFlushOnExit diagnosticsFlushOnExit;

        std::uint64_t wallNowNs()
        {
            return ros::WallTime::now().toNSec();
        }

        std::uint64_t simTimeUs(const ros::Time& time)
        {
            return static_cast<std::uint64_t>(time.toNSec() / 1000ULL);
        }

        bool ensureDiagnosticsOpen(const std::string& path)
        {
            if(diagnosticsStream.is_open()){
                return true;
            }
            diagnosticsStream.open(path, std::ios::out | std::ios::trunc);
            if(!diagnosticsStream.is_open()){
                ROS_ERROR("Unable to open LowCmd apply diagnostics CSV: %s", path.c_str());
                return false;
            }
            diagnosticsStream
                << "event,stage,joint,wall_time_ns,sim_time_us,sim_dt_us,period_us,"
                << "controller_update_sequence,physics_update_sequence,buffer_write_sequence,"
                << "command_sequence,previous_command_sequence,new_command,effective_application,"
                << "thread_id,receive_wall_time_ns,receive_sim_time_us,payload_hash,"
                << "mode,q,dq,tau,kp,kd\n";
            diagnosticsLastFlushWallNs = wallNowNs();
            return true;
        }

        void flushDiagnosticsLocked()
        {
            if(!diagnosticsStream.is_open() || diagnosticsBuffer.empty()){
                return;
            }
            for(const auto &line : diagnosticsBuffer){
                diagnosticsStream << line << '\n';
            }
            diagnosticsBuffer.clear();
            diagnosticsStream.flush();
            diagnosticsLastFlushWallNs = wallNowNs();
        }

        void appendDiagnosticsLine(const std::string &line, std::uint64_t nowWallNs)
        {
            diagnosticsBuffer.push_back(line);
            if(diagnosticsBuffer.size() >= kDiagnosticsFlushRows ||
               nowWallNs - diagnosticsLastFlushWallNs >= kDiagnosticsFlushIntervalNs){
                flushDiagnosticsLocked();
            }
        }

        std::uint64_t hashFloat(std::uint64_t seed, float value)
        {
            std::uint32_t bits = 0;
            static_assert(sizeof(bits) == sizeof(value), "float hash size mismatch");
            std::memcpy(&bits, &value, sizeof(bits));
            seed ^= static_cast<std::uint64_t>(bits) + 0x9e3779b97f4a7c15ULL +
                    (seed << 6U) + (seed >> 2U);
            return seed;
        }

        std::uint64_t payloadHash(const unitree_legged_msgs::MotorCmd &cmd)
        {
            std::uint64_t hash = 1469598103934665603ULL;
            hash ^= static_cast<std::uint64_t>(cmd.mode) + 0x9e3779b97f4a7c15ULL;
            hash = hashFloat(hash, cmd.q);
            hash = hashFloat(hash, cmd.dq);
            hash = hashFloat(hash, cmd.tau);
            hash = hashFloat(hash, cmd.Kp);
            hash = hashFloat(hash, cmd.Kd);
            return hash;
        }

        std::uint64_t threadIdHash()
        {
            return static_cast<std::uint64_t>(
                std::hash<std::thread::id>{}(std::this_thread::get_id()));
        }

        int commandSpinnerThreadCount()
        {
            int threads = 4;
            ros::param::param<int>("/lowcmd_command_spinner_threads", threads, threads);
            const char *env = std::getenv("LOWCMD_COMMAND_SPINNER_THREADS");
            if(env != nullptr && env[0] != '\0'){
                try {
                    threads = std::stoi(env);
                } catch (const std::exception&) {
                    ROS_WARN("Invalid LOWCMD_COMMAND_SPINNER_THREADS='%s'; using %d",
                             env, threads);
                }
            }
            return std::max(1, threads);
        }

        void ensureCommandCallbackSpinnerStarted()
        {
            std::call_once(commandCallbackSpinnerOnce, [](){
                const int threads = commandSpinnerThreadCount();
                commandCallbackSpinner.reset(
                    new ros::AsyncSpinner(static_cast<std::uint32_t>(threads),
                                          &commandCallbackQueue));
                commandCallbackSpinner->start();
                ROS_INFO("LowCmd command callback spinner started with %d thread(s).",
                         threads);
            });
        }

        bool envFlagEnabled(const char *name, bool fallback)
        {
            const char *value = std::getenv(name);
            if(value == nullptr || value[0] == '\0'){
                return fallback;
            }
            const std::string text(value);
            return text == "1" || text == "true" || text == "TRUE" ||
                   text == "yes" || text == "YES" || text == "on" ||
                   text == "ON";
        }

        std::string envString(const char *name, const std::string &fallback)
        {
            const char *value = std::getenv(name);
            return value == nullptr || value[0] == '\0' ? fallback : value;
        }

        double normalizedJointPosition(double position, const urdf::JointConstSharedPtr& jointUrdf)
        {
            if(jointUrdf && jointUrdf->type == urdf::Joint::REVOLUTE){
                return std::atan2(std::sin(position), std::cos(position));
            }
            return position;
        }
    }

    UnitreeJointController::UnitreeJointController(){
        memset(&lastCmd, 0, sizeof(unitree_legged_msgs::MotorCmd));
        memset(&lastState, 0, sizeof(unitree_legged_msgs::MotorState));
        memset(&servoCmd, 0, sizeof(ServoCmd));
        diagnostics_enabled = false;
        diagnostics_target_joint = false;
        received_command_sequence = 0;
        applied_command_sequence = 0;
        controller_update_sequence = 0;
        diagnostic_buffer_write_sequence = 0;
    }

    UnitreeJointController::~UnitreeJointController(){
        sub_ft.shutdown();
        sub_cmd.shutdown();
    }

    void UnitreeJointController::setTorqueCB(const geometry_msgs::WrenchStampedConstPtr& msg)
    {
        if(isHip) sensor_torque = msg->wrench.torque.x;
        else sensor_torque = msg->wrench.torque.y;
        // printf("sensor torque%f\n", sensor_torque);
    }

    void UnitreeJointController::setCommandCB(const unitree_legged_msgs::MotorCmdConstPtr& msg)
    {
        lastCmd.mode = msg->mode;
        lastCmd.q = msg->q;
        lastCmd.Kp = msg->Kp;
        lastCmd.dq = msg->dq;
        lastCmd.Kd = msg->Kd;
        lastCmd.tau = msg->tau;
        StampedMotorCmd stampedCmd;
        stampedCmd.cmd = lastCmd;
        stampedCmd.sequence = ++received_command_sequence;
        stampedCmd.receive_wall_time_ns = wallNowNs();
        stampedCmd.receive_sim_time_us = simTimeUs(ros::Time::now());
        writeCommandDiagnostics("T1_CALLBACK_ENTRY", stampedCmd, ros::Time::now(),
                                ros::Duration(0.0), true, false);
	        // the writeFromNonRT can be used in RT, if you have the guarantee that
	        //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
	        //  * there is only one single rt thread
	        command.writeFromNonRT(stampedCmd);
        ++diagnostic_buffer_write_sequence;
        writeCommandDiagnostics("T2_BUFFER_WRITE", stampedCmd, ros::Time::now(),
                                ros::Duration(0.0), true, false);
    }

    // Controller initialization in non-realtime
    bool UnitreeJointController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        isHip = false;
        isThigh = false;
        isCalf = false;
        // rqtTune = false;
        sensor_torque = 0;
        name_space = n.getNamespace();
        if (!n.getParam("joint", joint_name)){
            ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
            return false;
        }
        std::string diagnosticsJoint = "FR_hip_joint";
        std::string diagnosticsPath = "logs/lowcmd_apply_timing.csv";
        ros::param::param<bool>("/lowcmd_apply_diagnostics_enabled",
                                diagnostics_enabled, false);
        ros::param::param<std::string>("/lowcmd_apply_diagnostics_joint",
                                       diagnosticsJoint, diagnosticsJoint);
        ros::param::param<std::string>("/lowcmd_apply_diagnostics_path",
                                       diagnosticsPath, diagnosticsPath);
        diagnostics_enabled = envFlagEnabled("LOWCMD_APPLY_DIAGNOSTICS_ENABLED",
                                             diagnostics_enabled);
        diagnosticsJoint = envString("LOWCMD_APPLY_DIAGNOSTICS_JOINT",
                                     diagnosticsJoint);
        diagnosticsPath = envString("LOWCMD_APPLY_DIAGNOSTICS_PATH",
                                    diagnosticsPath);
        diagnostics_target_joint = diagnostics_enabled &&
            (diagnosticsJoint == joint_name || diagnosticsJoint == name_space);
        if(diagnostics_target_joint){
            std::lock_guard<std::mutex> lock(diagnosticsMutex);
            if(ensureDiagnosticsOpen(diagnosticsPath)){
                ROS_INFO("LowCmd apply diagnostics enabled for %s -> %s",
                         joint_name.c_str(), diagnosticsPath.c_str());
            }else{
                diagnostics_target_joint = false;
            }
        }
        
        // load pid param from ymal only if rqt need 
        // if(rqtTune) {
#ifdef rqtTune
            // Load PID Controller using gains set on parameter server
            if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
                return false;
#endif
        // }

        urdf::Model urdf; // Get URDF info about joint
        if (!urdf.initParamWithNodeHandle("robot_description", n)){
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf){
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }
        if(joint_name == "FR_hip_joint" || joint_name == "FL_hip_joint" || joint_name == "RR_hip_joint" || joint_name == "RL_hip_joint"){
            isHip = true;
        }
        if(joint_name == "FR_calf_joint" || joint_name == "FL_calf_joint" || joint_name == "RR_calf_joint" || joint_name == "RL_calf_joint"){
            isCalf = true;
        }        
        joint = robot->getHandle(joint_name);

        // Start command subscriber
        sub_ft = n.subscribe(name_space + "/" +"joint_wrench", 1, &UnitreeJointController::setTorqueCB, this);
        ensureCommandCallbackSpinnerStarted();
        ros::SubscribeOptions commandSubscribeOptions =
            ros::SubscribeOptions::create<unitree_legged_msgs::MotorCmd>(
                "command", 1,
                boost::bind(&UnitreeJointController::setCommandCB, this,
                            boost::placeholders::_1),
                ros::VoidPtr(), &commandCallbackQueue);
        commandSubscribeOptions.transport_hints = ros::TransportHints().tcpNoDelay();
        sub_cmd = n.subscribe(commandSubscribeOptions);

        // pub_state = n.advertise<unitree_legged_msgs::MotorState>(name_space + "/state", 20); 
        // Start realtime state publisher
        controller_state_publisher_.reset(
            new realtime_tools::RealtimePublisher<unitree_legged_msgs::MotorState>(n, name_space + "/state", 1));        

        return true;
    }

    void UnitreeJointController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_controller_.setGains(p,i,d,i_max,i_min,antiwindup);
    }

    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
    {
        pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
    }

    void UnitreeJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        bool dummy;
        pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
    }

    // Controller startup in realtime
    void UnitreeJointController::starting(const ros::Time& time)
    {
        // lastCmd.Kp = 0;
        // lastCmd.Kd = 0;
        double init_pos = normalizedJointPosition(joint.getPosition(), joint_urdf);
        lastCmd.q = init_pos;
        lastState.q = init_pos;
        lastCmd.dq = 0;
        lastState.dq = 0;
        lastCmd.tau = 0;
        lastState.tauEst = 0;
        lastStampedCmd.cmd = lastCmd;
        lastStampedCmd.sequence = 0;
        lastStampedCmd.receive_wall_time_ns = wallNowNs();
        lastStampedCmd.receive_sim_time_us = simTimeUs(time);
        command.initRT(lastStampedCmd);
        applied_command_sequence = 0;
        controller_update_sequence = 0;
        diagnostic_buffer_write_sequence = 0;

        pid_controller_.reset();
    }

    // Controller update loop in realtime
    void UnitreeJointController::update(const ros::Time& time, const ros::Duration& period)
    {
        double currentPos, currentVel, calcTorque;
        ++controller_update_sequence;
        lastStampedCmd = *(command.readFromRT());
        lastCmd = lastStampedCmd.cmd;
        const bool newCommand = lastStampedCmd.sequence != applied_command_sequence;
        writeCommandDiagnostics("T3_CONTROLLER_READ", lastStampedCmd, time, period,
                                newCommand, false);
        applied_command_sequence = lastStampedCmd.sequence;

        // set command data
        if(lastCmd.mode == PMSM) {
            servoCmd.pos = lastCmd.q;
            positionLimits(servoCmd.pos);
            servoCmd.posStiffness = lastCmd.Kp;
            if(fabs(lastCmd.q - PosStopF) < 0.00001){
                servoCmd.posStiffness = 0;
            }
            servoCmd.vel = lastCmd.dq;
            velocityLimits(servoCmd.vel);
            servoCmd.velStiffness = lastCmd.Kd;
            if(fabs(lastCmd.dq - VelStopF) < 0.00001){
                servoCmd.velStiffness = 0;
            }
            servoCmd.torque = lastCmd.tau;
            effortLimits(servoCmd.torque);
        }
        if(lastCmd.mode == BRAKE) {
            servoCmd.posStiffness = 0;
            servoCmd.vel = 0;
            servoCmd.velStiffness = 20;
            servoCmd.torque = 0;
            effortLimits(servoCmd.torque);
        }

        // } else {
        //     servoCmd.posStiffness = 0;
        //     servoCmd.velStiffness = 5;
        //     servoCmd.torque = 0;
        // }
        
        // rqt set P D gains
        // if(rqtTune) {
#ifdef rqtTune
            double i, i_max, i_min;
            getGains(servoCmd.posStiffness,i,servoCmd.velStiffness,i_max,i_min);
#endif
        // } 

        // Gazebo 11 may report a bounded revolute joint using an equivalent
        // positive angle (for example +3.59 rad instead of -2.69 rad).  The
        // A1 command and kinematics contracts use the signed URDF interval.
        currentPos = normalizedJointPosition(joint.getPosition(), joint_urdf);
        currentVel = computeVel(currentPos, (double)lastState.q, (double)lastState.dq, period.toSec());
        calcTorque = computeTorque(currentPos, currentVel, servoCmd);      
        effortLimits(calcTorque);

        joint.setCommand(calcTorque);
        writeCommandDiagnostics("T4_JOINT_APPLY", lastStampedCmd, time, period,
                                newCommand, true);

        lastState.q = currentPos;
        lastState.dq = currentVel;
        // lastState.tauEst = calcTorque;
        // lastState.tauEst = sensor_torque;
        lastState.tauEst = joint.getEffort();

        // pub_state.publish(lastState);
        // publish state
        if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
            controller_state_publisher_->msg_.q = lastState.q;
            controller_state_publisher_->msg_.dq = lastState.dq;
            controller_state_publisher_->msg_.tauEst = lastState.tauEst;
            controller_state_publisher_->unlockAndPublish();
        }

        // printf("sensor torque%f\n", sensor_torque);

        // if(joint_name == "wrist1_joint") printf("wrist1 setp:%f  getp:%f t:%f\n", servoCmd.pos, currentPos, calcTorque);
    }

    // Controller stopping in realtime
    void UnitreeJointController::stopping(){}

    void UnitreeJointController::writeCommandDiagnostics(const char *stage,
                                                         const StampedMotorCmd &cmd,
                                                         const ros::Time &time,
                                                         const ros::Duration &period,
                                                         bool newCommand,
                                                         bool effectiveApplication)
    {
        if(!diagnostics_target_joint){
            return;
        }
        const std::uint64_t nowWallNs = wallNowNs();
        std::ostringstream line;
        line << "LOWCMD_TRACE" << ',' << stage << ',' << joint_name << ','
             << nowWallNs << ',' << simTimeUs(time) << ','
             << static_cast<std::int64_t>(period.toSec() * 1000000.0) << ','
             << static_cast<std::uint64_t>(period.toSec() * 1000000.0) << ','
             << controller_update_sequence << ',' << controller_update_sequence << ','
             << diagnostic_buffer_write_sequence << ',' << cmd.sequence << ','
             << applied_command_sequence << ',' << (newCommand ? 1 : 0) << ','
             << (effectiveApplication ? 1 : 0) << ',' << threadIdHash() << ','
             << cmd.receive_wall_time_ns << ',' << cmd.receive_sim_time_us << ','
             << payloadHash(cmd.cmd) << ',' << static_cast<int>(cmd.cmd.mode) << ','
             << std::setprecision(9) << cmd.cmd.q << ',' << cmd.cmd.dq << ','
             << cmd.cmd.tau << ',' << cmd.cmd.Kp << ',' << cmd.cmd.Kd;
        std::lock_guard<std::mutex> lock(diagnosticsMutex);
        if(!diagnosticsStream.is_open()){
            return;
        }
        appendDiagnosticsLine(line.str(), nowWallNs);
    }

    void UnitreeJointController::positionLimits(double &position)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(position, joint_urdf->limits->lower, joint_urdf->limits->upper);
    }

    void UnitreeJointController::velocityLimits(double &velocity)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(velocity, -joint_urdf->limits->velocity, joint_urdf->limits->velocity);
    }

    void UnitreeJointController::effortLimits(double &effort)
    {
        if (joint_urdf->type == urdf::Joint::REVOLUTE || joint_urdf->type == urdf::Joint::PRISMATIC)
            clamp(effort, -joint_urdf->limits->effort, joint_urdf->limits->effort);
    }

} // namespace

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(unitree_legged_control::UnitreeJointController, controller_interface::ControllerBase);
