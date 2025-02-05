#ifndef _TOPO_REPLAN_FSM_H_
#define _TOPO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline/Bspline.h>
#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace fast_planner {
class TopoReplanFSM {
public:
  enum PLAN_STEP { FULL, REFINE };

private:
  const char *_label = "[topo_fsm] ";

  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET, REFENCE_PATH };
  const std::string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_NEW"};
  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_; // 1 mannual select, 2 hard code
  uint _raplan_max_failed = 10;
  double replan_distance_threshold_, replan_time_threshold_;
  double waypoints_[50][3];
  int waypoint_num_;
  bool act_map_;
  bool _enable_viz;

  /* planning data */
  bool trigger_, have_target_, have_odom_, collide_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
  Eigen::Vector3d target_point_, end_vel_;                       // target state
  int current_wp_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber waypoint_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;
  ros::Publisher _wait_goal_pub;

  /* helper functions */
  bool callTopologicalTraj(PLAN_STEP step); // topo path guided gradient-based
                                      // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, const char *pos_call);

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent &e);
  void checkCollisionCallback(const ros::TimerEvent &e);
  void frontierCallback(const ros::TimerEvent &e);
  void waypointCallback(const nav_msgs::PathConstPtr &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

  /* visualize new trajectories */
  void visualization();

public:
  TopoReplanFSM(/* args */) {}
  ~TopoReplanFSM() {}

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner

#endif