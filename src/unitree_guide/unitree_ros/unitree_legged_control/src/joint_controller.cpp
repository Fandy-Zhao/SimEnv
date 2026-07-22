/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// #include "unitree_legged_control/joint_controller.h"
#include "joint_controller.h"
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <mutex>
#include <string>
#include <pluginlib/class_list_macros.h>
#include <ros/param.h>

// #define rqtTune // use rqt or not

namespace unitree_legged_control 
{
    namespace
    {
        std::mutex diagnosticsMutex;
        std::ofstream diagnosticsStream;

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
                << "event,joint,wall_time_ns,sim_time_us,sim_dt_us,period_us,"
                << "controller_update_sequence,command_sequence,previous_command_sequence,"
                << "new_command,receive_wall_time_ns,receive_sim_time_us,"
                << "mode,q,dq,tau,kp,kd\n";
            return true;
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
        // the writeFromNonRT can be used in RT, if you have the guarantee that
        //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
        //  * there is only one single rt thread
        command.writeFromNonRT(stampedCmd);
        writeCommandDiagnostics("CMD_RECEIVE", stampedCmd, ros::Time::now(),
                                ros::Duration(0.0), true);
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
        sub_cmd = n.subscribe("command", 1, &UnitreeJointController::setCommandCB, this,
                              ros::TransportHints().tcpNoDelay());

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
        if(newCommand || lastStampedCmd.sequence == 0){
            writeCommandDiagnostics("CMD_APPLY", lastStampedCmd, time, period, newCommand);
        }
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

    void UnitreeJointController::writeCommandDiagnostics(const char *event,
                                                         const StampedMotorCmd &cmd,
                                                         const ros::Time &time,
                                                         const ros::Duration &period,
                                                         bool newCommand)
    {
        if(!diagnostics_target_joint){
            return;
        }
        std::lock_guard<std::mutex> lock(diagnosticsMutex);
        if(!diagnosticsStream.is_open()){
            return;
        }
        diagnosticsStream << event << ',' << joint_name << ',' << wallNowNs() << ','
                          << simTimeUs(time) << ','
                          << static_cast<std::int64_t>(period.toSec() * 1000000.0) << ','
                          << static_cast<std::uint64_t>(period.toSec() * 1000000.0) << ','
                          << controller_update_sequence << ',' << cmd.sequence << ','
                          << applied_command_sequence << ',' << (newCommand ? 1 : 0) << ','
                          << cmd.receive_wall_time_ns << ',' << cmd.receive_sim_time_us << ','
                          << static_cast<int>(cmd.cmd.mode) << ',' << cmd.cmd.q << ','
                          << cmd.cmd.dq << ',' << cmd.cmd.tau << ',' << cmd.cmd.Kp << ','
                          << cmd.cmd.Kd << '\n';
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
