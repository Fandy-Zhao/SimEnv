/*
exploration_with_graph_planner.cpp
the interface for drrt planner

Created and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
 */

#include <chrono>
#include <deque>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_datatypes.h>

#include "dsvplanner/clean_frontier_srv.h"
#include "dsvplanner/dsvplanner_srv.h"
#include "graph_planner/GraphPlannerCommand.h"
#include "graph_planner/GraphPlannerStatus.h"

using namespace std::chrono;
#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

geometry_msgs::Point wayPoint;
geometry_msgs::Point wayPoint_pre;
geometry_msgs::Point goal_point;
geometry_msgs::Point home_point;
graph_planner::GraphPlannerCommand graph_planner_command;
std_msgs::Float32 effective_time;
std_msgs::Float32 total_time;

bool simulation = false;    // control whether use graph planner to follow path
bool begin_signal = false;  // trigger the planner
bool gp_in_progress = false;
bool wp_state = false;
bool return_home = false;
bool odom_received = false;
double current_odom_x = 0;
double current_odom_y = 0;
double current_odom_z = 0;
double previous_odom_x = 0;
double previous_odom_y = 0;
double previous_odom_z = 0;
double dtime = 0.0;
double init_x = 2;
double init_y = 0;
double init_z = 2;
double init_time = 2;
double return_home_threshold = 1.5;
double robot_moving_threshold = 6;
bool skip_initial_motion = false;
double initialization_timeout = 5.0;
double initialization_distance_tolerance = 0.25;
double bootstrap_min_displacement = 0.5;
bool continue_after_initialization_timeout = true;
double movement_window = 2.0;
double movement_distance_threshold = 0.08;
double stuck_timeout = 15.0;
int max_replans_before_frontier_clean = 2;
bool single_floor_enabled = false;
double floor_reference_z = 0.0;
double max_goal_z_deviation = 0.20;
bool floor_reference_initialized = false;
int min_exploration_iterations = 5;
int valid_goal_count = 0;
bool map_warmed_up = false;
double min_useful_goal_distance = 0.4;
int max_premature_completion_retries = 3;
int consecutive_planner_unready_count = 0;
bool bootstrap_recovery_attempted = false;
std::string map_frame = "map";
std::string waypoint_topic = "/way_point";
std::string cmd_vel_topic = "/cmd_vel";
std::string gp_command_topic = "/graph_planner_command";
std::string effective_plan_time_topic = "/runtime";
std::string total_plan_time_topic = "/totaltime";
std::string gp_status_topic = "/graph_planner_status";
std::string odom_topic = "/state_estimation";
std::string begin_signal_topic = "/start_exploring";
std::string stop_signal_topic = "/stop_exploring";
std::string planner_service_name = "drrtPlannerSrv";
std::string clean_frontier_service_name = "cleanFrontierSrv";

struct OdomSample
{
  double stamp;
  double x;
  double y;
  double z;
};

std::deque<OdomSample> odom_history;
double last_movement_time = 0.0;

tf::StampedTransform transformToMap;

steady_clock::time_point plan_start;
steady_clock::time_point plan_over;
steady_clock::duration time_span;

ros::Publisher waypoint_pub;
ros::Publisher gp_command_pub;
ros::Publisher effective_plan_time_pub;
ros::Publisher total_plan_time_pub;
ros::Subscriber gp_status_sub;
ros::Subscriber waypoint_sub;
ros::Subscriber odom_sub;
ros::Subscriber begin_signal_sub;
ros::Publisher stop_signal_pub;

void gp_status_callback(const graph_planner::GraphPlannerStatus::ConstPtr& msg)
{
  if (msg->status == graph_planner::GraphPlannerStatus::STATUS_IN_PROGRESS)
    gp_in_progress = true;
  else
  {
    gp_in_progress = false;
  }
}

void waypoint_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  wayPoint = msg->point;
  wp_state = true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_odom_x = msg->pose.pose.position.x;
  current_odom_y = msg->pose.pose.position.y;
  current_odom_z = msg->pose.pose.position.z;
  odom_received = true;
  const double stamp = msg->header.stamp.toSec() > 0.0 ? msg->header.stamp.toSec() : ros::Time::now().toSec();

  if (!floor_reference_initialized) {
    floor_reference_z = current_odom_z;
    floor_reference_initialized = true;
  }

  odom_history.push_back({stamp, current_odom_x, current_odom_y, current_odom_z});
  while (!odom_history.empty() && stamp - odom_history.front().stamp > movement_window) {
    odom_history.pop_front();
  }

  transformToMap.setOrigin(
      tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
}

void begin_signal_callback(const std_msgs::Bool::ConstPtr& msg)
{
  begin_signal = msg->data;
}

bool robotPositionChange()
{
  if (odom_history.size() < 2) return false;

  const OdomSample& first = odom_history.front();
  const OdomSample& last = odom_history.back();
  double dist = sqrt((last.x - first.x) * (last.x - first.x) +
                     (last.y - first.y) * (last.y - first.y) +
                     (last.z - first.z) * (last.z - first.z));
  if (dist >= movement_distance_threshold) {
    last_movement_time = last.stamp;
    return true;
  }
  return false;
}

void enforceSingleFloor(geometry_msgs::Point& point)
{
  if (!single_floor_enabled || !floor_reference_initialized) return;
  const double min_z = floor_reference_z - max_goal_z_deviation;
  const double max_z = floor_reference_z + max_goal_z_deviation;
  if (point.z < min_z) point.z = min_z;
  else if (point.z > max_z) point.z = max_z;
}

double goalDistanceXY(const geometry_msgs::Point& goal)
{
  return hypot(goal.x - current_odom_x, goal.y - current_odom_y);
}

bool isUsefulGoal(const geometry_msgs::Point& goal)
{
  return std::isfinite(goal.x) && std::isfinite(goal.y) && std::isfinite(goal.z) &&
         goalDistanceXY(goal) > min_useful_goal_distance;
}

void publishBootstrapFailure(const std::string& reason)
{
  ROS_ERROR("BOOTSTRAP_EXPLORATION_FAILED reason=%s valid_goals=%d unready_count=%d recovery_attempted=%d",
            reason.c_str(), valid_goal_count, consecutive_planner_unready_count,
            bootstrap_recovery_attempted ? 1 : 0);
  std_msgs::Bool stop_exploring;
  stop_exploring.data = true;
  stop_signal_pub.publish(stop_exploring);
}

bool initializationMotion(bool capture_home, const char* stage)
{
  if (capture_home)
  {
    home_point.x = current_odom_x;
    home_point.y = current_odom_y;
    home_point.z = current_odom_z;
  }

  geometry_msgs::Point motion_start;
  motion_start.x = current_odom_x;
  motion_start.y = current_odom_y;
  motion_start.z = current_odom_z;

  const double init_distance = sqrt(init_x * init_x + init_y * init_y + init_z * init_z);
  if (skip_initial_motion || init_distance <= initialization_distance_tolerance) {
    ROS_INFO("DSV initialization skipped: stage=%s skip=%d init_distance=%.3f tolerance=%.3f",
             stage, skip_initial_motion ? 1 : 0, init_distance, initialization_distance_tolerance);
    return true;
  }

  tf::Vector3 vec_init(init_x, init_y, init_z);
  tf::Vector3 vec_goal;
  vec_goal = transformToMap * vec_init;
  geometry_msgs::PointStamped wp;
  wp.header.frame_id = map_frame;
  wp.header.stamp = ros::Time::now();
  wp.point.x = vec_goal.x();
  wp.point.y = vec_goal.y();
  wp.point.z = vec_goal.z();
  enforceSingleFloor(wp.point);
  ROS_INFO("DSV_BOOTSTRAP_START stage=%s start=(%.3f,%.3f,%.3f) waypoint=(%.3f,%.3f,%.3f) required_displacement=%.3f",
           stage, motion_start.x, motion_start.y, motion_start.z, wp.point.x, wp.point.y, wp.point.z,
           bootstrap_min_displacement);

  ros::Duration(0.5).sleep();  // wait for sometime to make sure waypoint can be
                               // published properly

  waypoint_pub.publish(wp);
  bool wp_ongoing = true;
  const double init_start_time = ros::Time::now().toSec();
  while (wp_ongoing)
  {  // Keep publishing initial waypoint until the robot
    // reaches that point
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    wp.header.stamp = ros::Time::now();
    waypoint_pub.publish(wp);
    double dist = sqrt((wp.point.x - current_odom_x) * (wp.point.x - current_odom_x) +
                       (wp.point.y - current_odom_y) * (wp.point.y - current_odom_y));
    double displacement = hypot(motion_start.x - current_odom_x, motion_start.y - current_odom_y);
    if (dist <= initialization_distance_tolerance && displacement >= bootstrap_min_displacement)
      wp_ongoing = false;
    if (ros::Time::now().toSec() - init_start_time >= init_time && displacement >= bootstrap_min_displacement)
      wp_ongoing = false;
    if (ros::Time::now().toSec() - init_start_time >= initialization_timeout) {
      ROS_ERROR("DSV_BOOTSTRAP_TIMEOUT stage=%s timeout=%.2f dist=%.3f displacement=%.3f required=%.3f continue=%d",
                stage, initialization_timeout, dist, displacement, bootstrap_min_displacement,
                continue_after_initialization_timeout ? 1 : 0);
      return continue_after_initialization_timeout;
    }
  }
  const double displacement = hypot(motion_start.x - current_odom_x, motion_start.y - current_odom_y);
  ROS_INFO("DSV_BOOTSTRAP_SUCCESS stage=%s pose=(%.3f,%.3f,%.3f) displacement=%.3f",
           stage, current_odom_x, current_odom_y, current_odom_z, displacement);
  return true;
}

bool retryOrRecoverBootstrap(const std::string& reason)
{
  consecutive_planner_unready_count++;
  if (consecutive_planner_unready_count <= max_premature_completion_retries)
  {
    ROS_WARN("DSV_MAP_WARMUP_RETRY reason=%s retry=%d/%d valid_goals=%d min=%d",
             reason.c_str(), consecutive_planner_unready_count, max_premature_completion_retries,
             valid_goal_count, min_exploration_iterations);
    return true;
  }

  if (!bootstrap_recovery_attempted)
  {
    bootstrap_recovery_attempted = true;
    ROS_WARN("DSV_MAP_WARMUP_RECOVERY reason=%s retries_exhausted=%d",
             reason.c_str(), max_premature_completion_retries);
    if (initializationMotion(false, "warmup_recovery"))
    {
      consecutive_planner_unready_count = 0;
      return true;
    }
  }

  publishBootstrapFailure(reason);
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");
  
  nhPrivate.getParam("simulation", simulation);
  nhPrivate.getParam("/interface/dtime", dtime);
  nhPrivate.getParam("/interface/initX", init_x);
  nhPrivate.getParam("/interface/initY", init_y);
  nhPrivate.getParam("/interface/initZ", init_z);
  nhPrivate.getParam("/interface/initTime", init_time);
  nhPrivate.getParam("/interface/returnHomeThres", return_home_threshold);
  nhPrivate.getParam("/interface/robotMovingThres", robot_moving_threshold);
  nhPrivate.getParam("/interface/skipInitialMotion", skip_initial_motion);
  nhPrivate.getParam("/interface/initializationTimeout", initialization_timeout);
  nhPrivate.getParam("/interface/initializationDistanceTolerance", initialization_distance_tolerance);
  nhPrivate.getParam("/interface/bootstrapMinDisplacement", bootstrap_min_displacement);
  nhPrivate.getParam("/interface/continueAfterInitializationTimeout", continue_after_initialization_timeout);
  nhPrivate.getParam("/interface/movementWindow", movement_window);
  nhPrivate.getParam("/interface/movementDistanceThreshold", movement_distance_threshold);
  nhPrivate.getParam("/interface/stuckTimeout", stuck_timeout);
  nhPrivate.getParam("/interface/maxReplansBeforeFrontierClean", max_replans_before_frontier_clean);
  nhPrivate.getParam("/interface/tfFrame", map_frame);
  nhPrivate.getParam("/interface/autoExp", begin_signal);
  nhPrivate.getParam("/interface/waypointTopic", waypoint_topic);
  nhPrivate.getParam("/interface/cmdVelTopic", cmd_vel_topic);
  nhPrivate.getParam("/interface/graphPlannerCommandTopic", gp_command_topic);
  nhPrivate.getParam("/interface/effectivePlanTimeTopic", effective_plan_time_topic);
  nhPrivate.getParam("/interface/totalPlanTimeTopic", total_plan_time_topic);
  nhPrivate.getParam("/interface/gpStatusTopic", gp_status_topic);
  nhPrivate.getParam("/interface/odomTopic", odom_topic);
  nhPrivate.getParam("/interface/beginSignalTopic", begin_signal_topic);
  nhPrivate.getParam("/interface/stopSignalTopic", stop_signal_topic);
  nhPrivate.getParam("/planner/plannerServiceName", planner_service_name);
  nhPrivate.getParam("/planner/cleanFrontierServiceName", clean_frontier_service_name);
  nhPrivate.getParam("/single_floor/enabled", single_floor_enabled);
  nhPrivate.getParam("/single_floor/max_goal_z_deviation", max_goal_z_deviation);
  nhPrivate.getParam("/single_floor/min_exploration_iterations", min_exploration_iterations);
  nhPrivate.getParam("/single_floor/min_useful_goal_distance", min_useful_goal_distance);
  nhPrivate.getParam("/single_floor/max_premature_completion_retries", max_premature_completion_retries);

  waypoint_pub = nh.advertise<geometry_msgs::PointStamped>(waypoint_topic, 5);
  gp_command_pub = nh.advertise<graph_planner::GraphPlannerCommand>(gp_command_topic, 1);
  effective_plan_time_pub = nh.advertise<std_msgs::Float32>(effective_plan_time_topic, 1);
  total_plan_time_pub = nh.advertise<std_msgs::Float32>(total_plan_time_topic, 1);
  gp_status_sub = nh.subscribe<graph_planner::GraphPlannerStatus>(gp_status_topic, 1, gp_status_callback);
  waypoint_sub = nh.subscribe<geometry_msgs::PointStamped>(waypoint_topic, 1, waypoint_callback);
  odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, odom_callback);
  begin_signal_sub = nh.subscribe<std_msgs::Bool>(begin_signal_topic, 1, begin_signal_callback);
  stop_signal_pub = nh.advertise<std_msgs::Bool>(stop_signal_topic, 1);

  ros::Duration(1.0).sleep();
  ros::spinOnce();

  while (!odom_received)
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    ROS_INFO("Waiting for Odometry");
  }

  while (!begin_signal)
  {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    ROS_INFO("Waiting for exploration start signal");
  }

  ROS_INFO("Starting the planner: Performing initialization motion");
  if (!initializationMotion(true, "startup")) {
    publishBootstrapFailure("startup_motion_timeout");
    return 1;
  }
  ros::Duration(1.0).sleep();

  std::cout << std::endl << "\033[1;32mExploration Started\033[0m\n" << std::endl;
  total_time.data = 0;
  plan_start = steady_clock::now();
  // Start planning: The planner is called and the computed goal point sent to
  // the graph planner.
  int iteration = 0;
  while (ros::ok())
  {
    if (!return_home)
    {
      if (iteration != 0)
      {
        for (int i = 0; i < 8; i++)
        {
          printf(cursup);
          printf(cursclean);
        }
      }
      std::cout << "Planning iteration " << iteration << std::endl;
      dsvplanner::dsvplanner_srv planSrv;
      dsvplanner::clean_frontier_srv cleanSrv;
      planSrv.request.header.stamp = ros::Time::now();
      planSrv.request.header.seq = iteration;
      planSrv.request.header.frame_id = map_frame;
      if (ros::service::call(planner_service_name, planSrv))
      {
        if (planSrv.response.goal.size() == 0)
        {  // usually the size should be 1 if planning successfully
          if (!retryOrRecoverBootstrap("planner_no_goal"))
            return 2;
          ros::Duration(1.0).sleep();
          continue;
        }

        if (planSrv.response.mode.data == 2)
        {
          if (!map_warmed_up && valid_goal_count < min_exploration_iterations)
          {
            if (!retryOrRecoverBootstrap("premature_mode2"))
              return 2;
            ros::Duration(0.5).sleep();
            continue;
          }
          return_home = true;
          goal_point = home_point;
          enforceSingleFloor(goal_point);
          std::cout << std::endl << "\033[1;32mExploration completed, returning home\033[0m" << std::endl << std::endl;
          effective_time.data = 0;
          effective_plan_time_pub.publish(effective_time);
        }
        else
        {
          return_home = false;
          goal_point = planSrv.response.goal[0];
          enforceSingleFloor(goal_point);
          if (!isUsefulGoal(goal_point))
          {
            ROS_ERROR("DSV_DEGENERATE_GOAL planner_stage=exploration goal=(%.3f,%.3f,%.3f) robot=(%.3f,%.3f,%.3f) goal_distance=%.3f min_distance=%.3f",
                      goal_point.x, goal_point.y, goal_point.z, current_odom_x, current_odom_y, current_odom_z,
                      goalDistanceXY(goal_point), min_useful_goal_distance);
            if (!retryOrRecoverBootstrap("degenerate_mode1_goal"))
              return 2;
            continue;
          }
          consecutive_planner_unready_count = 0;
          valid_goal_count++;
          if (valid_goal_count >= min_exploration_iterations)
            map_warmed_up = true;
          plan_over = steady_clock::now();
          time_span = plan_over - plan_start;
          effective_time.data = float(time_span.count()) * steady_clock::period::num / steady_clock::period::den;
          effective_plan_time_pub.publish(effective_time);
        }
        total_time.data += effective_time.data;
        total_plan_time_pub.publish(total_time);

        if (!simulation)
        {  // when not in simulation mode, the robot will go to
           // the goal point according to graph planner
          graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
          graph_planner_command.location = goal_point;
          gp_command_pub.publish(graph_planner_command);
          ros::Duration(dtime).sleep();  // give sometime to graph planner for
                                         // searching path to goal point
          ros::spinOnce();               // update gp_in_progree
          int count = 200;
          int replan_count = 0;
          previous_odom_x = current_odom_x;
          previous_odom_y = current_odom_y;
          previous_odom_z = current_odom_z;
          last_movement_time = ros::Time::now().toSec();
          while (gp_in_progress)
          {                              // if the waypoint keep the same for 20
                                         // (200*0.1)
            ros::Duration(0.1).sleep();  // seconds, then give up the goal
            wayPoint_pre = wayPoint;
            ros::spinOnce();
            bool robotMoving = robotPositionChange();
            if (robotMoving)
            {
              count = 200;
              replan_count = 0;
            }
            else
            {
              count--;
            }
            const double now = ros::Time::now().toSec();
            if (now - last_movement_time >= stuck_timeout)
            {
              replan_count++;
              ROS_WARN("DSV movement window stuck: window=%.2fs threshold=%.3fm stuck=%.2fs replan=%d/%d",
                       movement_window, movement_distance_threshold, now - last_movement_time, replan_count,
                       max_replans_before_frontier_clean);
              if (replan_count <= max_replans_before_frontier_clean)
              {
                graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                graph_planner_command.location = goal_point;
                gp_command_pub.publish(graph_planner_command);
                last_movement_time = now;
                count = 200;
                continue;
              }
              count = 0;
            }
            if (count <= 0)
            {  // when the goal point cannot be reached, clean
               // its correspoinding frontier if there is
              cleanSrv.request.header.stamp = ros::Time::now();
              cleanSrv.request.header.frame_id = map_frame;
              ROS_WARN("DSV cleaning frontier after repeated stuck/replan failures");
              ros::service::call(clean_frontier_service_name, cleanSrv);
              ros::Duration(0.1).sleep();
              break;
            }
          }

          graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_DISABLE;
          gp_command_pub.publish(graph_planner_command);
        }
        else
        {  // simulation mode is used when testing this planning algorithm
           // with bagfiles where robot will
          // not move to the planned goal. When in simulation mode, robot will
          // keep replanning every two seconds
          for (size_t i = 0; i < planSrv.response.goal.size(); i++)
          {
            graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
            graph_planner_command.location = planSrv.response.goal[i];
            gp_command_pub.publish(graph_planner_command);
            ros::Duration(2).sleep();
            break;
          }
        }
        plan_start = steady_clock::now();
      }
      else
      {
        std::cout << "Cannot call drrt planner." << std::flush;

        ros::Duration(1.0).sleep();
      }
      iteration++;
    }
    else
    {
      ros::spinOnce();
      if (fabs(current_odom_x - home_point.x) + fabs(current_odom_y - home_point.y) +
              fabs(current_odom_z - home_point.z) <=
          return_home_threshold)
      {
        printf(cursclean);
        std::cout << "\033[1;32mReturn home completed\033[0m" << std::endl;
        printf(cursup);
        std_msgs::Bool stop_exploring;
        stop_exploring.data = true;
        stop_signal_pub.publish(stop_exploring);
      }
      else
      {
        while (!gp_in_progress)
        {
          ros::spinOnce();
          ros::Duration(2.0).sleep();

          graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
          graph_planner_command.location = goal_point;
          gp_command_pub.publish(graph_planner_command);
        }
      }
      ros::Duration(0.1).sleep();
    }
  }
}
