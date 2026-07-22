/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
#include "interface/IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <string>
#include "nav_msgs/Odometry.h"
#include "rosgraph_msgs/Clock.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/WrenchStamped.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>

class IOROS : public IOInterface{
public:
IOROS();
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
void recvStateOnly(LowlevelState *state) override;
void publishCmdOnly(const LowlevelCmd *cmd) override;
bool hasFullStateFeedback() const override;
std::uint64_t stateSequence() const override;
std::uint64_t stateStampUs() const override;
bool getPolicyInputSnapshot(PolicyInputSnapshot &snapshot) const override;

private:
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);
void updateMotorState(int index, const unitree_legged_msgs::MotorState& msg);
ros::NodeHandle _nm;
ros::Subscriber _servo_sub[12], _imu_sub, _foot_states_sub[4], _foot_force_sub[4],
                _base_w_sub, _base_t_sub, _time_sub, joy_sub;
ros::Publisher _servo_pub[12];
unitree_legged_msgs::LowCmd _lowCmd;
unitree_legged_msgs::LowState _lowState;
std::string _robot_name;
std::array<std::atomic_bool, 12> _joint_state_received;
std::atomic_bool _imu_received;
std::atomic_bool _base_world_received;
std::array<std::atomic<float>, 4> _foot_force;
std::array<std::atomic<std::uint64_t>, 4> _foot_force_wall_stamp_ns;
	std::array<std::atomic<std::uint64_t>, 4> _foot_force_callback_sequence;
	std::array<std::atomic<std::uint64_t>, 4> _foot_force_sim_time_us;
std::atomic<std::uint64_t> _state_sequence{0};
std::atomic<std::uint64_t> _state_stamp_us{0};
std::uint64_t _lowcmd_sequence = 0;
mutable std::mutex _policy_snapshot_mutex;
PolicyInputSnapshot _policy_input_snapshot;

//repeated functions for multi-thread
void initRecv();
void initSend();

//Callback functions for ROS
void imuCallback(const sensor_msgs::Imu & msg);
void updateFootForce(int index, const geometry_msgs::WrenchStamped& msg);
void FRfootForceCallback(const geometry_msgs::WrenchStamped& msg);
void FLfootForceCallback(const geometry_msgs::WrenchStamped& msg);
void RRfootForceCallback(const geometry_msgs::WrenchStamped& msg);
void RLfootForceCallback(const geometry_msgs::WrenchStamped& msg);

void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);


void timeCallback(const rosgraph_msgs::Clock& msg);
void baseWorldCallback(const nav_msgs::Odometry& msg);
void baseTrunkCallback(const nav_msgs::Odometry& msg);
void FL_footCallback(const nav_msgs::Odometry& msg);
void FR_footCallback(const nav_msgs::Odometry& msg);
void RL_footCallback(const nav_msgs::Odometry& msg);
void RR_footCallback(const nav_msgs::Odometry& msg);
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
};

#endif  // IOROS_H

#endif  // COMPILE_WITH_ROS
