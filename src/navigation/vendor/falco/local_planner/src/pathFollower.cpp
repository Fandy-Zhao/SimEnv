#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
bool enableHeadingSpeedSchedule = true;
bool enableFollowerDiagnostics = false;
double diagnosticThrottleSec = 1.0;
double straightHeadingDeg = 10.0;
double normalTurnHeadingDeg = 30.0;
double sharpTurnHeadingDeg = 60.0;
double straightSpeed = 0.8;
double turnSpeed = 0.6;
double sharpTurnSpeed = 0.3;
double maxErrorSpeed = 0.2;
double maxYawAccel = 1.0;
double inputTimeout = 0.5;

// Turn-before-forward for A1: rear goals are handled by turning in place
// rather than reversing.  Only applies in autonomy mode.
double turnInPlaceThresholdDeg = 90.0;    // |heading| above this → linear=0, turn only
double forwardEnableThresholdDeg = 35.0;  // |heading| below this → normal forward speed
double rearGoalSlowSpeed = 0.05;          // linear speed cap in the 60°–90° band
bool allowReverse = false;                // false = turn in place for rear goals
bool reverseEscapeEnabled = true;         // allow brief reverse to escape being stuck
double reverseEscapeMaxDuration = 1.5;    // max seconds of reverse escape

// Internal state for reverse-escape tracking
double reverseEscapeStartTime = 0.0;
bool inReverseEscape = false;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double pathTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

nav_msgs::Path path;

double clampValue(double value, double minValue, double maxValue)
{
  return std::max(minValue, std::min(maxValue, value));
}

double interpolateSpeed(double value, double x0, double y0, double x1, double y1)
{
  if (x1 <= x0) return y1;
  double ratio = clampValue((value - x0) / (x1 - x0), 0.0, 1.0);
  return y0 + ratio * (y1 - y0);
}

double scheduledSpeedForHeading(double absHeadingError)
{
  if (!enableHeadingSpeedSchedule) return maxSpeed * joySpeed;

  const double straightHeading = straightHeadingDeg * PI / 180.0;
  const double normalTurnHeading = normalTurnHeadingDeg * PI / 180.0;
  const double sharpTurnHeading = sharpTurnHeadingDeg * PI / 180.0;
  const double turnInPlaceRad = turnInPlaceThresholdDeg * PI / 180.0;
  const double forwardEnableRad = forwardEnableThresholdDeg * PI / 180.0;

  // Turn-in-place band: |heading| > turnInPlaceThreshold → zero linear speed
  if (absHeadingError > turnInPlaceRad) {
    return 0.0;
  }
  // Near-rear band: 60° < |heading| <= turnInPlaceThreshold → crawl speed
  if (absHeadingError > sharpTurnHeading) {
    return rearGoalSlowSpeed;
  }
  // Slowed forward band: forwardEnableRad < |heading| <= sharpTurnHeading
  if (absHeadingError > forwardEnableRad) {
    return interpolateSpeed(absHeadingError, forwardEnableRad, sharpTurnSpeed,
                            sharpTurnHeading, rearGoalSlowSpeed);
  }
  // Normal bands below forwardEnableRad
  if (absHeadingError <= straightHeading) {
    return straightSpeed;
  }
  if (absHeadingError <= normalTurnHeading) {
    return interpolateSpeed(absHeadingError, straightHeading, straightSpeed,
                            normalTurnHeading, turnSpeed);
  }
  if (absHeadingError <= forwardEnableRad) {
    return interpolateSpeed(absHeadingError, normalTurnHeading, turnSpeed,
                            forwardEnableRad, sharpTurnSpeed);
  }
  return maxErrorSpeed;
}

double rampToward(double current, double target, double maxStep)
{
  if (current < target) return std::min(current + maxStep, target);
  if (current > target) return std::max(current - maxStep, target);
  return current;
}

void odomHandler(const nav_msgs::Odometry::ConstPtr& odomIn)
{
  odomTime = odomIn->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = odomIn->header.stamp.toSec();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = odomIn->header.stamp.toSec();
  }
}

void pathHandler(const nav_msgs::Path::ConstPtr& pathIn)
{
  int pathSize = pathIn->poses.size();
  if (pathSize <= 0) {
    path.poses.clear();
    pathInit = false;
    pathTime = ros::Time::now().toSec();
    return;
  }
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
  pathTime = ros::Time::now().toSec();
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyTime = ros::Time::now().toSec();

  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }
}

void speedHandler(const std_msgs::Float32::ConstPtr& speed)
{
  double speedTime = ros::Time::now().toSec();

  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::Int8::ConstPtr& stop)
{
  safetyStop = stop->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathFollower");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("lookAheadDis", lookAheadDis);
  nhPrivate.getParam("yawRateGain", yawRateGain);
  nhPrivate.getParam("stopYawRateGain", stopYawRateGain);
  nhPrivate.getParam("maxYawRate", maxYawRate);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("maxAccel", maxAccel);
  nhPrivate.getParam("switchTimeThre", switchTimeThre);
  nhPrivate.getParam("dirDiffThre", dirDiffThre);
  nhPrivate.getParam("stopDisThre", stopDisThre);
  nhPrivate.getParam("slowDwnDisThre", slowDwnDisThre);
  nhPrivate.getParam("useInclRateToSlow", useInclRateToSlow);
  nhPrivate.getParam("inclRateThre", inclRateThre);
  nhPrivate.getParam("slowRate1", slowRate1);
  nhPrivate.getParam("slowRate2", slowRate2);
  nhPrivate.getParam("slowTime1", slowTime1);
  nhPrivate.getParam("slowTime2", slowTime2);
  nhPrivate.getParam("useInclToStop", useInclToStop);
  nhPrivate.getParam("inclThre", inclThre);
  nhPrivate.getParam("stopTime", stopTime);
  nhPrivate.getParam("noRotAtStop", noRotAtStop);
  nhPrivate.getParam("noRotAtGoal", noRotAtGoal);
  nhPrivate.getParam("autonomyMode", autonomyMode);
  nhPrivate.getParam("autonomySpeed", autonomySpeed);
  nhPrivate.getParam("joyToSpeedDelay", joyToSpeedDelay);
  nhPrivate.getParam("enableHeadingSpeedSchedule", enableHeadingSpeedSchedule);
  nhPrivate.getParam("enableFollowerDiagnostics", enableFollowerDiagnostics);
  nhPrivate.getParam("diagnosticThrottleSec", diagnosticThrottleSec);
  nhPrivate.getParam("straightHeadingDeg", straightHeadingDeg);
  nhPrivate.getParam("normalTurnHeadingDeg", normalTurnHeadingDeg);
  nhPrivate.getParam("sharpTurnHeadingDeg", sharpTurnHeadingDeg);
  nhPrivate.getParam("straightSpeed", straightSpeed);
  nhPrivate.getParam("turnSpeed", turnSpeed);
  nhPrivate.getParam("sharpTurnSpeed", sharpTurnSpeed);
  nhPrivate.getParam("maxErrorSpeed", maxErrorSpeed);
  nhPrivate.getParam("maxYawAccel", maxYawAccel);
  nhPrivate.getParam("inputTimeout", inputTimeout);

  // Turn-before-forward parameters for A1 rear-goal handling
  nhPrivate.getParam("turnInPlaceThresholdDeg", turnInPlaceThresholdDeg);
  nhPrivate.getParam("forwardEnableThresholdDeg", forwardEnableThresholdDeg);
  nhPrivate.getParam("rearGoalSlowSpeed", rearGoalSlowSpeed);
  nhPrivate.getParam("allowReverse", allowReverse);
  nhPrivate.getParam("reverseEscapeEnabled", reverseEscapeEnabled);
  nhPrivate.getParam("reverseEscapeMaxDuration", reverseEscapeMaxDuration);

  // Clamp new params to reasonable bounds
  turnInPlaceThresholdDeg = clampValue(turnInPlaceThresholdDeg, 60.0, 135.0);
  forwardEnableThresholdDeg = clampValue(forwardEnableThresholdDeg, 15.0, turnInPlaceThresholdDeg - 10.0);
  rearGoalSlowSpeed = clampValue(rearGoalSlowSpeed, 0.0, 0.15);

  straightSpeed = clampValue(straightSpeed, 0.0, maxSpeed);
  turnSpeed = clampValue(turnSpeed, 0.0, straightSpeed);
  sharpTurnSpeed = clampValue(sharpTurnSpeed, 0.0, turnSpeed);
  maxErrorSpeed = clampValue(maxErrorSpeed, 0.0, sharpTurnSpeed);

  ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/state_estimation", 5, odomHandler);

  ros::Subscriber subPath = nh.subscribe<nav_msgs::Path> ("/path", 5, pathHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  ros::Subscriber subSpeed = nh.subscribe<std_msgs::Float32> ("/speed", 5, speedHandler);

  ros::Subscriber subStop = nh.subscribe<std_msgs::Int8> ("/stop", 5, stopHandler);

  ros::Publisher pubSpeed = nh.advertise<geometry_msgs::TwistStamped> ("/cmd_vel", 5);
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "vehicle";

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      // Save original heading error before twoWayDrive modifies it
      const float originalDirDiff = dirDiff;
      const float absOriginalDirDiff = fabs(originalDirDiff);
      const double turnInPlaceRad = turnInPlaceThresholdDeg * PI / 180.0;

      if (twoWayDrive) {
        double time = ros::Time::now().toSec();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          if (allowReverse) {
            navFwd = false;
            switchTime = time;
          } else if (reverseEscapeEnabled && !inReverseEscape) {
            // Enter brief reverse escape only when stuck and reverse is enabled
            inReverseEscape = true;
            reverseEscapeStartTime = time;
            navFwd = false;
            switchTime = time;
          }
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
          inReverseEscape = false;
        }

        // Exit reverse escape after max duration
        if (inReverseEscape && time - reverseEscapeStartTime > reverseEscapeMaxDuration) {
          navFwd = true;
          switchTime = time;
          inReverseEscape = false;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      const float absDirDiff = fabs(dirDiff);
      if (autonomyMode) {
        joySpeed2 = scheduledSpeedForHeading(absDirDiff);
      }
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      // Turn-in-place for rear goals: when reverse is not allowed and the
      // waypoint is behind the robot, force linear speed to zero and only
      // allow yaw rotation.  The speed schedule already returns 0 for
      // |heading| > turnInPlaceThreshold, but this is a hard gate.
      if (!allowReverse && autonomyMode && !inReverseEscape &&
          absOriginalDirDiff > turnInPlaceRad) {
        joySpeed2 = 0.0;
      }

      double desiredYawRate;
      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) desiredYawRate = -stopYawRateGain * dirDiff;
      else desiredYawRate = -yawRateGain * dirDiff;

      if (desiredYawRate > maxYawRate * PI / 180.0) desiredYawRate = maxYawRate * PI / 180.0;
      else if (desiredYawRate < -maxYawRate * PI / 180.0) desiredYawRate = -maxYawRate * PI / 180.0;
      vehicleYawRate = rampToward(vehicleYawRate, desiredYawRate, maxYawAccel / 100.0);

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;

      if ((enableHeadingSpeedSchedule || fabs(dirDiff) < dirDiffThre) && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }

      double nowTime = ros::Time::now().toSec();
      bool inputFresh = (nowTime - odomTime <= inputTimeout) && (nowTime - pathTime <= inputTimeout);
      if (!inputFresh) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ros::Time().fromSec(odomTime);
        if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.twist.linear.x = 0;
        else cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        pubSpeed.publish(cmd_vel);
        if (enableFollowerDiagnostics) {
          const char* speedStage = "straight";
          if (absDirDiff > sharpTurnHeadingDeg * PI / 180.0) speedStage = "large_heading";
          else if (absDirDiff > normalTurnHeadingDeg * PI / 180.0) speedStage = "sharp_turn";
          else if (absDirDiff > straightHeadingDeg * PI / 180.0) speedStage = "turn";
          ROS_INFO_THROTTLE(diagnosticThrottleSec,
                            "falco_follower_diag heading_error_deg=%.1f stage=%s target_linear=%.3f raw_linear=%.3f "
                            "raw_angular=%.3f max_angular=%.3f end_dis=%.3f waypoint_dis=%.3f input_fresh=%d safety_stop=%d "
                            "turn_in_place=%d allow_reverse=%d reverse_escape=%d",
                            absDirDiff * 180.0 / PI, speedStage, joySpeed3, cmd_vel.twist.linear.x,
                            cmd_vel.twist.angular.z, maxYawRate * PI / 180.0, endDis, dis, inputFresh ? 1 : 0,
                            safetyStop,
                            (!allowReverse && absOriginalDirDiff > turnInPlaceRad) ? 1 : 0,
                            allowReverse ? 1 : 0,
                            inReverseEscape ? 1 : 0);
        }

        pubSkipCount = pubSkipNum;
      }
    } else {
      vehicleSpeed = 0;
      vehicleYawRate = 0;
      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.angular.z = 0;
        pubSpeed.publish(cmd_vel);
        pubSkipCount = pubSkipNum;
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
