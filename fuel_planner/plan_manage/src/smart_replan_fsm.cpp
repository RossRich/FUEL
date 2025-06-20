#include <plan_manage/smart_replan_fsm.hpp>

namespace fast_planner {
void SmartReplanFsm::init(ros::NodeHandle &nh) {
  have_target_ = false;
  have_odom_ = false;
  collide_ = false;
  _is_stop_req = false;
  _odom_time_stamp = ros::Time(0);
  _wait_pub_timer = ros::Time(0);
  _heartbeat_pub_timer = ros::Time(0);

  exec_state_ = FSM_EXEC_STATE::INIT;

  /*  fsm param  */
  nh.param("fsm/act_map", act_map_, false);
  nh.param("fsm/enable_viz", _enable_viz, false);
  _emergency_stop_dist = 1.2;

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  auto &ad = planner_manager_->agents_data;
  ad.init(0, 5);
  ros::NodeHandle glob;
  glob.param("sys_id", ad.drone_id, 0);
  ROS_ASSERT_MSG(ad.drone_id > 0, "Invalid agent id. %i <= 0", ad.drone_id);

  /* callback */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &SmartReplanFsm::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.03), &SmartReplanFsm::check_safety, this);
  // frontier_timer_ = nh.createTimer(ros::Duration(0.1), &SmartReplanFsm::frontierCallback, this);

  _stop_srv = nh.advertiseService("/planning/stop", &SmartReplanFsm::stop_srv, this);

  _agent_traj_sub0 = nh.subscribe("/planning/agent_traj_sub0", 50, &SmartReplanFsm::agent_traj_callback0, this);
  _agent_traj_sub1 = nh.subscribe("/planning/agent_traj_sub1", 50, &SmartReplanFsm::agent_traj_callback1, this);
  waypoint_sub_ = nh.subscribe("/planning/waypoint", 1, &SmartReplanFsm::waypointCallback, this);
  path_sub_ = nh.subscribe("/planning/path", 1, &SmartReplanFsm::pathCallback, this);
  odom_sub_ = nh.subscribe("/planning/odom_world", 30, &SmartReplanFsm::odometryCallback, this);

  _heartbeat_pub = nh.advertise<std_msgs::Empty>("/planning/heartbeat", 5);
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 20);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 20);
  bspline_pub_ = nh.advertise<planner_msgs::Bspline>("/planning/bspline", 20);
  _wait_goal_pub = nh.advertise<std_msgs::Empty>("/planning/wait", 5);
  _agent_traj_pub0 = nh.advertise<mavros_msgs::Trajectory>("/planning/agent_traj_pub0", 20);
  _agent_traj_pub1 = nh.advertise<mavros_msgs::Tunnel>("/planning/agent_traj_pub1", 20);
}

bool SmartReplanFsm::stop_srv(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  res.success = 1;
  res.message = "ok";
  _is_stop_req = true;
  return true;
}

void SmartReplanFsm::agent_traj_callback1(const mavros_msgs::TunnelConstPtr &tunnel_msg) {

  const uint8_t NUM_PT = 9U;
  const uint8_t agent_id = tunnel_msg->target_system;
  auto &agents_data = planner_manager_->agents_data;

  struct trag_t {
    float x[NUM_PT];
    float y[NUM_PT];
    float z[NUM_PT];
    float dt;
    uint8_t valid_pt;
  } bspline_struct;

  uint8_t *data_arr = reinterpret_cast<uint8_t *>(&bspline_struct);

  std::copy(tunnel_msg->payload.data(), tunnel_msg->payload.data() + tunnel_msg->payload_length, data_arr);

  points3d_t ctrl_pts;
  for (size_t i = 0; i < bspline_struct.valid_pt; ++i)
    ctrl_pts.push_back({bspline_struct.x[i], bspline_struct.y[i], bspline_struct.z[i]});

  if (ctrl_pts.size() < 3) return;

  std::string tgt_frame("earth");
  std::string src_frame("map_");
  src_frame += std::to_string(agent_id);

  try {
    for (auto &_ctrl_pt : ctrl_pts) {
      geometry_msgs::PointStamped tmp_pt = tf2::toMsg(tf2::Stamped<point3d_t>(_ctrl_pt, ros::Time::now(), src_frame));
      auto new_vector = _tf_buffer.transform(tmp_pt, tgt_frame);
      tf2::fromMsg(new_vector.point, _ctrl_pt);
    }
  } catch (const std::exception &e) {
    ROS_WARN_STREAM(_label << e.what());
    return;
  }

  Eigen::MatrixXd pos_pts(ctrl_pts.size(), 3);

  for (size_t i = 0; i < ctrl_pts.size(); ++i) {
    pos_pts(i, 0) = ctrl_pts.at(i).x();
    pos_pts(i, 1) = ctrl_pts.at(i).y();
    pos_pts(i, 2) = ctrl_pts.at(i).z();
  }

  auto &at = agents_data.trajs.at(agent_id - 1);

  at.setUniformBspline(pos_pts, planner_manager_->pp_.bspline_degree_, bspline_struct.dt);
  at.start_time_ = ros::Time::now().toSec();
  agents_data.receive_flags.at(agent_id - 1) = true;

  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    if (not planner_manager_->checkAgentCollision(agent_id)) {
      ROS_WARN("%sUnsafe fligth paths for %i and %i", _label, agents_data.drone_id, agent_id);
      changeFSMExecState(FSM_EXEC_STATE::REPLAN_TRAJ, "CHECKING TRAJ");
    }
  }

  // visualization_->drawBspline(at, 0.05, {0.5, 0.5, 0.5, 0.8});
}

void SmartReplanFsm::agent_traj_callback0(const hg_msgs::IsotopeTrajectoryConstPtr &agent_msg) {
  auto &agents_data = planner_manager_->agents_data;
  const auto &trajectory = agent_msg->trajectory;
  const auto &agent_id = agent_msg->sys_id;

  // size_t valid_pts = std::count(trajectory.point_valid.begin(), trajectory.point_valid.end(), 1);
  // Eigen::MatrixXd pos_pts(valid_pts, 3);

  std::array<const mavros_msgs::PositionTarget *, trajectory.point_valid.size()> pts_arr = {
      &trajectory.point_1, &trajectory.point_2, &trajectory.point_3, &trajectory.point_4, &trajectory.point_5};

  points3d_t ctrl_pts;
  for (size_t i = 0; i < pts_arr.size(); ++i) {
    if (not trajectory.point_valid.at(i)) continue;
    auto &_pt = pts_arr.at(i)->position;
    ctrl_pts.push_back({_pt.x, _pt.y, _pt.z});
  }

  if (ctrl_pts.size() < 3) return;

  Eigen::MatrixXd pos_pts(ctrl_pts.size(), 3);

  for (size_t i = 0; i < ctrl_pts.size(); ++i) {
    pos_pts(i, 0) = ctrl_pts.at(i).x();
    pos_pts(i, 1) = ctrl_pts.at(i).y();
    pos_pts(i, 2) = ctrl_pts.at(i).z();
  }

  auto &at = agents_data.trajs.at(agent_id - 1);
  // double dt = (ctrl_pts.at(1) - ctrl_pts.at(0)).norm();
  double dt = 0.8;
  at.setUniformBspline(pos_pts, planner_manager_->pp_.bspline_degree_, dt);
  at.start_time_ = trajectory.header.stamp.toSec();
  agents_data.receive_flags.at(agent_id - 1) = true;

  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    if (not planner_manager_->checkAgentCollision(agent_id)) {
      ROS_WARN("%sUnsafe fligth paths for %i and %i", _label, agents_data.drone_id, agent_id);
      changeFSMExecState(FSM_EXEC_STATE::REPLAN_TRAJ, "CHECKING TRAJ");
    }
  }

  // visualization_->drawBspline(at, 0.05, {0.5, 0.5, 0.5, 0.8});
}

void SmartReplanFsm::waypointCallback(const geometry_msgs::PoseStampedPtr &pose) {
  if (not have_odom_) return;

  auto &msg_pt = pose->pose;
  Eigen::Vector3d e_new_point(msg_pt.position.x, msg_pt.position.y, msg_pt.position.z);
  ROS_DEBUG_STREAM(_label << "New waypoint. " << e_new_point.transpose().format(vector3d_fmt));

  if (exec_state_ != WAIT_TARGET) {
    ROS_WARN_STREAM(_label << "Planner busy. Waypoint rejected");
    return;
  }

  /* близко к дрону */
  if ((odom_pos_ - e_new_point).norm() < 1.0) {
    ROS_INFO_STREAM(_label << "Position reached");
    return;
  }

  /* слишком низко */
  if (e_new_point.z() < 0.3) {
    ROS_ERROR_STREAM(_label << "Invalid height\n" << msg_pt.position);
    return;
  }

  vector<Eigen::Vector3d> global_wp;
  target_point_(0) = msg_pt.position.x;
  target_point_(1) = msg_pt.position.y;
  target_point_(2) = msg_pt.position.z;
  global_wp.push_back(target_point_);
  visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

  planner_manager_->setGlobalWaypoints(global_wp);
  have_target_ = true;
  _is_stop_req = false;
}

void SmartReplanFsm::pathCallback(const nav_msgs::PathConstPtr &msg) {
  if (not have_odom_) return;

  ROS_WARN_STREAM(_label << "Not implemented function");
  return;

  //< WARNING: Функция не обновлялась

  auto &msg_pt = msg->poses[0].pose;
  Eigen::Vector3d e_new_point(msg_pt.position.x, msg_pt.position.y, msg_pt.position.z);
  ROS_DEBUG("%sNew waypoint. x: %4.2f y: %4.2f z: %4.2f", _label, e_new_point.x(), e_new_point.y(), e_new_point.z());

  /* близко к дрону или ниже земли */
  if (e_new_point.z() < -0.1 or (odom_pos_ - e_new_point).norm() < 1.5) {
    ROS_ERROR_STREAM(_label << "Invalid point:\n" << msg_pt.position);
    return;
  }

  if (exec_state_ != WAIT_TARGET) {
    ROS_WARN_STREAM(_label << "Planner busy. Waypoint rejected");
    return;
  }

  vector<Eigen::Vector3d> global_wp;
  target_point_(0) = msg_pt.position.x;
  target_point_(1) = msg_pt.position.y;
  target_point_(2) = msg_pt.position.z;
  global_wp.push_back(target_point_);
  // visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

  planner_manager_->setGlobalWaypoints(global_wp);
  have_target_ = true;
  _is_stop_req = false;
} // namespace fast_planner

void SmartReplanFsm::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  _odom_time_stamp = msg->header.stamp + ros::Duration(2.0);
}

void SmartReplanFsm::changeFSMExecState(FSM_EXEC_STATE new_state, const char *pos_call) {
  ROS_DEBUG_NAMED("fsm", "%sTransition from %s to %s. Caller: %s", _label, state_str[int(exec_state_)].c_str(),
                  state_str[int(new_state)].c_str(), pos_call);

  exec_state_ = new_state;
}

void SmartReplanFsm::execFSMCallback(const ros::TimerEvent &e) {
  static uint _replan_num = 0;
  static uint failed_num = 0;
  const auto time_now = ros::Time::now();

  have_odom_ = time_now < _odom_time_stamp;

  if (time_now > _heartbeat_pub_timer) {
    _heartbeat_pub.publish(std_msgs::Empty());
    _heartbeat_pub_timer = time_now + ros::Duration(3);
  }

  switch (exec_state_) {
  case INIT: {
    if (have_odom_)
      changeFSMExecState(WAIT_TARGET, "FSM");
    else
      ROS_WARN_STREAM_THROTTLE(5, _label << "No odometry");
    break;
  }

  case WAIT_TARGET: {
    collide_ = false;
    if (have_target_) {
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    } else {
      if (time_now > _wait_pub_timer) {
        _wait_goal_pub.publish(std_msgs::Empty());
        _wait_pub_timer = ros::Time::now() + ros::Duration(5);
      }
    }
    break;
  }

  case GEN_NEW_TRAJ: {
    new_pub_.publish(std_msgs::Empty());

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
    start_yaw_(0) = atan2(rot_x(1), rot_x(0));
    start_yaw_(1) = start_yaw_(2) = 0.0;

    if (callPathPlanner(PLAN_STEP::FULL)) {
      changeFSMExecState(EXEC_TRAJ, "FSM");
    } else {
      ++failed_num;
      if (failed_num > _replan_max_failed) {
        failed_num = 0;
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
      } else {
        ROS_WARN("%sPlanning failed. Retrying... [%u/%u]", _label, failed_num, _replan_max_failed);
        ros::Duration(0.25).sleep();
      }
    }

    break;
  }

  case EXEC_TRAJ: {
    auto &pm = *planner_manager_;
    auto &global_data = pm.global_data_;
    // const auto exc_time_now = ros::Time::now();

    if (_is_stop_req) {
      // запрос на остановку движения
      changeFSMExecState(FSM_EXEC_STATE::STOP, "FSM");
    } else if (global_data.is_traj_end()) {
      have_target_ = false;
      changeFSMExecState(FSM_EXEC_STATE::WAIT_TARGET, "FSM");
    } else {
      // глобальная длиннее локальной
      bool is_glob_long = pm.global_data_.local_end_time_ < pm.global_data_.global_duration_;
      // прошел часть пути по локальной
      bool is_motion_start = (time_now - pm.local_data_.start_time_).toSec() > pm.local_data_.duration_ / 2.0;
      if (is_glob_long and is_motion_start) {
        ROS_DEBUG("%s[%u] Replan", _label, _replan_num);
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
    }

    break;
  }

  case REPLAN_TRAJ: {
    replan_pub_.publish(std_msgs::Empty());
    LocalTrajData &local_traj = planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - local_traj.start_time_).toSec();

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    // start_pt_ = local_traj.position_traj_.evaluateDeBoorT(t_cur);
    // start_vel_ = local_traj.velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = local_traj.acceleration_traj_.evaluateDeBoorT(t_cur);

    start_yaw_(0) = local_traj.yaw_traj_.evaluateDeBoorT(t_cur)[0];
    start_yaw_(1) = local_traj.yawdot_traj_.evaluateDeBoorT(t_cur)[0];
    start_yaw_(2) = local_traj.yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

    if (not callPathPlanner(PLAN_STEP::REFINE))
      ROS_DEBUG("%s[%u]Replan failed", _label, _replan_num); //< если возвращает false, то возможно путь был стерт
    // TODO: Если путь был стерт, то что дальше?

    changeFSMExecState(EXEC_TRAJ, "FSM");

    ++_replan_num;

    break;
  }

  case STOP: {
    new_pub_.publish(std_msgs::Empty());
    _is_stop_req = false;
    have_target_ = false;
    auto new_state = have_odom_ ? FSM_EXEC_STATE::WAIT_TARGET : FSM_EXEC_STATE::INIT;
    changeFSMExecState(new_state, "FSM");
    break;
  }
  }
}

void SmartReplanFsm::check_safety(const ros::TimerEvent &e) {
  if (exec_state_ == EXEC_TRAJ) {
    if (have_odom_) {
      double dist;
      collide_ = not planner_manager_->checkTrajCollision(dist);
      if (collide_) {
        ROS_WARN("%sCurrent traj %0.2f m to collision", _label, dist);
        if (dist < _emergency_stop_dist) {
          changeFSMExecState(STOP, "SAFETY");
          ROS_ERROR_STREAM(_label << "Stop. Collision detected");
        } else {
          changeFSMExecState(FSM_EXEC_STATE::REPLAN_TRAJ, "SAFETY");
        }
      }
    } else {
      changeFSMExecState(FSM_EXEC_STATE::STOP, "SAFETY");
      ROS_ERROR_STREAM(_label << "Stop. Odometry lost");
    }
  }
}

bool SmartReplanFsm::callPathPlanner(PLAN_STEP step) {
  auto &pm = *planner_manager_;
  auto &glob_data = pm.global_data_;

  if (step == PLAN_STEP::FULL)
    if (not pm.planGlobalTraj3(start_pt_, odom_orient_)) return false;

  const auto time_now = ros::Time::now();
  double local_traj_start = (time_now - glob_data.global_start_time_).toSec(); //< начало локальной траектории на глобальной

  if (not pm.planLocaTraj(local_traj_start, time_now)) return false;

  pm.refine_local_traj(time_now, collide_);

  if (!act_map_)
    pm.planYaw(start_yaw_);
  else
    pm.planYawActMap(start_yaw_);

  double dist = 0.0;
  if (step == PLAN_STEP::FULL and not pm.checkTrajCollision(dist) and dist < _emergency_stop_dist) return false;

  auto &local_traj_data = pm.local_data_;
  /* publish newest trajectory to server */

  /* publish traj */
  planner_msgs::Bspline bspline;
  bspline.order = pm.pp_.bspline_degree_;
  bspline.traj_id = local_traj_data.traj_id_;
  bspline.start_time = local_traj_data.start_time_;
  bspline.knot_span = local_traj_data.position_traj_.getKnotSpan();

  auto &_traj = local_traj_data.position_traj_;

  auto &ctr_pts = _traj.getControlPoint();
  for (size_t i = 0; i < ctr_pts.rows(); ++i) {
    geometry_msgs::Point _pt;
    _pt.x = ctr_pts(i, 0);
    _pt.y = ctr_pts(i, 1);
    _pt.z = ctr_pts(i, 2);
    bspline.pos_pts.push_back(std::move(_pt));
  }

  auto &knots = _traj.getKnot();
  for (size_t i = 0; i < knots.rows(); ++i)
    bspline.knots.push_back(knots(i));

  auto &yaw_pts = local_traj_data.yaw_traj_.getControlPoint();
  for (size_t i = 0; i < yaw_pts.rows(); ++i)
    bspline.yaw_pts.push_back(yaw_pts(i));

  bspline.yaw_dt = local_traj_data.yaw_traj_.getKnotSpan();

  bspline_pub_.publish(bspline);

  publish_trajectory(bspline, 1);

  if (_enable_viz) visualization();

  return true;
}

void SmartReplanFsm::publish_trajectory(const planner_msgs::Bspline &bspline, uint8_t method) {

  if (method == 0) {
    mavros_msgs::Trajectory traj;
    traj.header.frame_id = "map";
    traj.header.seq += 1;
    traj.header.stamp = ros::Time::now();
    traj.type = mavros_msgs::Trajectory::MAV_TRAJECTORY_REPRESENTATION_BEZIER;
    traj.point_valid.fill(0);
    traj.command.fill(UINT16_MAX);
    traj.time_horizon.fill(NAN);

    auto &ros_ctrl_pts = bspline.pos_pts;

    // игнорим первую точку. К моменту анализа траектории бпла там уже не будет
    auto first_pt = ros_ctrl_pts.cbegin() + 1;
    int pts_count = static_cast<int>(std::distance(first_pt, ros_ctrl_pts.cend()));
    pts_count = std::min(5, pts_count);

    std::array<mavros_msgs::PositionTarget *, 5U> target_point = {&traj.point_1, &traj.point_2, &traj.point_3, &traj.point_4,
                                                                  &traj.point_5};
    for (uint i = 0; i < pts_count; ++i) {
      target_point.at(i)->position = ros_ctrl_pts.at(i);
      traj.point_valid.at(i) = 1;
    }

    _agent_traj_pub0.publish(traj);
  } else if (method == 1) {

    const uint8_t NUM_PT = 9U;

    struct trag_t {
      float x[NUM_PT];
      float y[NUM_PT];
      float z[NUM_PT];
      float dt;
      uint8_t valid_pt;
    } bspline_struct;

    bspline_struct.dt = static_cast<float>(bspline.knot_span);
    bspline_struct.valid_pt = 0;

    for (size_t i = 0; i < NUM_PT; ++i) {
      bspline_struct.x[i] = NAN;
      bspline_struct.y[i] = NAN;
      bspline_struct.z[i] = NAN;
    }

    for (size_t i = 1, j = 0; i < bspline.pos_pts.size(); ++i, ++j) {
      auto &_pt = bspline.pos_pts.at(i); //< игнорирую первую точку
      bspline_struct.x[j] = static_cast<float>(_pt.x);
      bspline_struct.y[j] = static_cast<float>(_pt.y);
      bspline_struct.z[j] = static_cast<float>(_pt.z);
      bspline_struct.valid_pt += 1;
    }

    ROS_DEBUG_STREAM("SIZE: " << sizeof(bspline_struct));

    if (bspline_struct.valid_pt == 0) return;

    uint8_t *data_arr = reinterpret_cast<uint8_t *>(&bspline_struct);

    mavros_msgs::Tunnel msg;
    msg.target_system = 0;
    msg.target_component = 0;
    msg.payload_type = 300;
    msg.payload_length = sizeof(bspline_struct);
    std::copy_n(data_arr, sizeof(bspline_struct), msg.payload.begin());
    _agent_traj_pub1.publish(msg);
  }
}

void SmartReplanFsm::frontierCallback(const ros::TimerEvent &e) {
  if (!have_odom_) return;
  planner_manager_->searchFrontier(odom_pos_);
  // visualization_->drawFrontier(planner_manager_->plan_data_.frontiers_);
}

void SmartReplanFsm::visualization() {
  GlobalTrajData &global_data = planner_manager_->global_data_;
  MidPlanData &plan_data = planner_manager_->plan_data_;
  LocalTrajData *local_traj = &planner_manager_->local_data_;

  const auto poli_traj_color = Eigen::Vector4d(255 / 255.0, 0 / 255.0, 0 / 255.0, 1.0);
  const auto spline_traj_color = Eigen::Vector4d(77 / 255.0, 77 / 255.0, 169 / 255.0, 1);
  const auto ctrl_pt_color = Eigen::Vector4d(49 / 255.0, 80 / 255.0, 119 / 255.0, 1);

  visualization_->drawPolynomialTraj(global_data.global_traj_, 0.05, poli_traj_color);
  visualization_->drawBspline(local_traj->position_traj_, 0.05, spline_traj_color, true, 0.1, ctrl_pt_color);

  // const auto color1 = Eigen::Vector4d(210 / 255.0, 0 / 255.0, 98 / 255.0, 1);
  // const auto color2 = Eigen::Vector4d(214 / 255.0, 88 / 255.0, 159 / 255.0, 1);
  // const auto color3 = Eigen::Vector4d(196 / 255.0, 228 / 255.0, 255 / 255.0, 1);

  // visualization_->drawTopoGraph(plan_data.topo_graph_, 0.08, 0.05, color1, color2, color3);

  // visualization_->drawBsplinesPhase2(plan_data.topo_traj_pos2_, 0.08);
  // visualization_->drawViewConstraint(plan_data.view_cons_);

  // visualization_->drawYawTraj(local_traj->position_traj_, local_traj->yaw_traj_, plan_data.dt_yaw_);
}

} // namespace fast_planner
