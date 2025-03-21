#if !defined(_SMART_REPLAN_FSM_)
#define _SMART_REPLAN_FSM_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline_opt/bspline_optimizer.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_manage/planner_manager.h>
#include <planner_msgs/AgentTraj.h>
#include <planner_msgs/Bspline.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <traj_utils/planning_visualization.h>
#include <vector>
#include <visualization_msgs/Marker.h>

namespace fast_planner {
class SmartReplanFsm {
public:
  enum PLAN_STEP { FULL, REFINE };

private:
  const char *_label = "[smart_fsm] ";

  /* ---------- flag ---------- */
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET, REFENCE_PATH };
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, STOP };
  const std::string state_str[6] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "STOP"};
  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  uint _replan_max_failed = 10;
  double _emergency_stop_dist = 0.0;
  bool act_map_;
  bool _enable_viz;

  /* planning data */
  bool have_target_, have_odom_, collide_;
  bool _is_stop_req;
  
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
  Eigen::Vector3d target_point_, end_vel_;                       // target state

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;

  ros::ServiceServer _stop_srv;

  ros::Subscriber waypoint_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber _agent_traj_sub;

  ros::Publisher _wait_goal_pub;
  ros::Publisher new_pub_;
  ros::Publisher bspline_pub_;
  ros::Publisher replan_pub_;
  ros::Publisher _agent_traj_pub;

  /* helper functions */
  bool callPathPlanner(PLAN_STEP step);
  void changeFSMExecState(FSM_EXEC_STATE new_state, const char *pos_call);

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent &e);
  void checkCollisionCallback(const ros::TimerEvent &e);
  void frontierCallback(const ros::TimerEvent &e);
  void pathCallback(const nav_msgs::PathConstPtr &msg);
  void waypointCallback(const geometry_msgs::PoseStampedPtr &);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void agent_traj_callback(const planner_msgs::AgentTrajConstPtr &agent_traj);
  bool stop_srv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

  /* visualize new trajectories */
  void visualization();

public:
  SmartReplanFsm() {}
  ~SmartReplanFsm() {}
  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace fast_planner

#endif // _SMART_REPLAN_FSM_
