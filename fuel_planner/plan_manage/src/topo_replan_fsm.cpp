
#include <plan_manage/topo_replan_fsm.h>

namespace fast_planner {
void TopoReplanFSM::init(ros::NodeHandle &nh) {
  current_wp_ = 0;
  have_target_ = false;
  have_odom_ = false;
  trigger_ = false;
  collide_ = false;
  exec_state_ = FSM_EXEC_STATE::INIT;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_time_threshold_, -1.0);
  nh.param("fsm/thresh_no_replan", replan_distance_threshold_, -1.0);
  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  nh.param("fsm/act_map", act_map_, false);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &TopoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &TopoReplanFSM::checkCollisionCallback, this);
  // frontier_timer_ = nh.createTimer(ros::Duration(0.1), &TopoReplanFSM::frontierCallback, this);

  waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &TopoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &TopoReplanFSM::odometryCallback, this);

  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 20);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 20);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 20);
  _wait_goal_pub = nh.advertise<std_msgs::Empty>("/planning/wait", 5);
}

void TopoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg) {
  if (not have_odom_) return;
  auto &msg_pt = msg->poses[0].pose;
  Eigen::Vector3d e_new_point(msg_pt.position.x, msg_pt.position.y, msg_pt.position.z);
  ROS_DEBUG("%sNew waypoint. x: %d y: %d z: %d", _label, e_new_point.x(), e_new_point.y(), e_new_point.z());

  /* близко к дрону или ниже земли */
  if (e_new_point.z() < -0.1 or (odom_pos_ - e_new_point).norm() < 1.5) {
    ROS_ERROR_STREAM(_label << "Invalid point: " << msg_pt.position);
    return;
  }

  if (exec_state_ != WAIT_TARGET) {
    if (_waypoints_queue.size() > 3) _waypoints_queue.pop();
    _waypoints_queue.push(std::move(e_new_point));
    ROS_WARN_STREAM(_label << "Planner busy. Waypoint moved to queue");
    return;
  }

  vector<Eigen::Vector3d> global_wp;
  if (target_type_ == TARGET_TYPE::REFENCE_PATH) {
    for (int i = 0; i < waypoint_num_; ++i) {
      Eigen::Vector3d pt;
      pt(0) = waypoints_[i][0];
      pt(1) = waypoints_[i][1];
      pt(2) = waypoints_[i][2];
      global_wp.push_back(pt);
    }
  } else {
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
      target_point_(0) = msg_pt.position.x;
      target_point_(1) = msg_pt.position.y;
      target_point_(2) = msg_pt.position.z;
    } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
      target_point_(0) = waypoints_[current_wp_][0];
      target_point_(1) = waypoints_[current_wp_][1];
      target_point_(2) = waypoints_[current_wp_][2];

      current_wp_ = (current_wp_ + 1) % waypoint_num_;
      std::cout << "preset: " << target_point_.transpose() << std::endl;
    }
    global_wp.push_back(target_point_);
    visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  }

  planner_manager_->setGlobalWaypoints(global_wp);
  have_target_ = true;
  trigger_ = true;
}

void TopoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
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

  have_odom_ = true;
}

void TopoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, const char *pos_call) {
  ROS_DEBUG_NAMED("fsm", "%sTransition from %s to %s. Caller: %s", _label, state_str[int(exec_state_)].c_str(),
                  state_str[int(new_state)].c_str(), pos_call);

  exec_state_ = new_state;
}

void TopoReplanFSM::execFSMCallback(const ros::TimerEvent &e) {
  static ros::Time wait_pub_timer = ros::Time::now() + ros::Duration(1);

  switch (exec_state_) {
  case INIT: {
    if (have_odom_)
      changeFSMExecState(WAIT_TARGET, "FSM");
    else
      ROS_WARN_STREAM_THROTTLE(5, _label << "No odometry");
    break;
  }

  case WAIT_TARGET: {
    if (_waypoints_queue.empty()) {
      if (ros::Time::now() > wait_pub_timer) {
        _wait_goal_pub.publish(std_msgs::Empty());
        wait_pub_timer = ros::Time::now() + ros::Duration(1);
      }
    } else {
      std::vector<Eigen::Vector3d> waypoints;
      for (size_t i = 0; i < _waypoints_queue.size(); ++i) {
        waypoints.emplace_back(_waypoints_queue.front());
        _waypoints_queue.pop();
      }
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
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

    /* topo path finding and optimization */
    if (callTopologicalTraj(1))
      changeFSMExecState(EXEC_TRAJ, "FSM");
    else
      ROS_WARN_STREAM(_label << "Planning failed. Retrying...");

    break;
  }

  case EXEC_TRAJ: {
    /* determine if need to replan */
    GlobalTrajData *global_data = &planner_manager_->global_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - global_data->global_start_time_).toSec();

    if (t_cur > global_data->global_duration_ - 0.1) {
      // если осталось двигаться по траектории 0.1 сек
      changeFSMExecState(WAIT_TARGET, "FSM");
    } else {
      LocalTrajData *local_traj = &planner_manager_->local_data_;
      t_cur = (time_now - local_traj->start_time_).toSec();

      if (t_cur > replan_time_threshold_) {
        if (!global_data->localTrajReachTarget()) {
          ROS_DEBUG_STREAM(_label << "Replan: periodic call");
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        } else {
          Eigen::Vector3d cur_pos = local_traj->position_traj_.evaluateDeBoorT(t_cur);
          Eigen::Vector3d end_pos = local_traj->position_traj_.evaluateDeBoorT(local_traj->duration_);
          if ((cur_pos - end_pos).norm() > replan_distance_threshold_) {
            ROS_DEBUG_STREAM(_label << "Replan: periodic call");
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        }
      }
    }
    break;
  }

  case REPLAN_TRAJ: {
    LocalTrajData &local_traj = planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - local_traj.start_time_).toSec();

    start_pt_ = local_traj.position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = local_traj.velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = local_traj.acceleration_traj_.evaluateDeBoorT(t_cur);

    start_yaw_(0) = local_traj.yaw_traj_.evaluateDeBoorT(t_cur)[0];
    start_yaw_(1) = local_traj.yawdot_traj_.evaluateDeBoorT(t_cur)[0];
    start_yaw_(2) = local_traj.yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

    if (not callTopologicalTraj(2)) ROS_DEBUG_STREAM(_label << "Replan failed");

    changeFSMExecState(EXEC_TRAJ, "FSM");

    break;
  }

  case REPLAN_NEW: {
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    /* inform server */
    new_pub_.publish(std_msgs::Empty());

    // bool success = callSearchAndOptimization();
    bool success = callTopologicalTraj(1);
    if (success) {
      changeFSMExecState(EXEC_TRAJ, "FSM");
    } else {
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }

    break;
  }
  }
}

void TopoReplanFSM::checkCollisionCallback(const ros::TimerEvent &e) {
  LocalTrajData &local_traj = planner_manager_->local_data_;

  /* ---------- check goal safety ---------- */
  // if (have_target_)
  if (false) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_
                      ? edt_env->evaluateCoarseEDT(target_point_, /* time to program start */ local_traj.duration_)
                      : edt_env->evaluateCoarseEDT(target_point_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;

      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {
            new_x = target_point_(0) + r * cos(theta / 57.3);
            new_y = target_point_(1) + r * sin(theta / 57.3);
            new_z = target_point_(2) + nz;
            Eigen::Vector3d new_pt(new_x, new_y, new_z);

            dist = planner_manager_->pp_.dynamic_ ? edt_env->evaluateCoarseEDT(new_pt,
                                                                               /* time to program start */ local_traj.duration_)
                                                  : edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset target_point_ */
              goal(0) = new_x;
              goal(1) = new_y;
              goal(2) = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        target_point_ = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_NEW, "SAFETY");
        }

        visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == EXEC_TRAJ) {
    double dist;
    collide_ = planner_manager_->checkTrajCollision(dist);
    if (collide_) {
      ROS_WARN("%sCurrent traj %lf m to collision", _label, dist);
      if (dist > 1.0) {
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        ROS_WARN_STREAM(_label << "Replan. Collision detected");
      } else {
        new_pub_.publish(std_msgs::Empty()); //< остановка
        changeFSMExecState(WAIT_TARGET, "SAFETY");
        ROS_ERROR_STREAM(_label << "Stop. Collision detected");
      }
    }
  }
}

void TopoReplanFSM::frontierCallback(const ros::TimerEvent &e) {
  if (!have_odom_) return;
  planner_manager_->searchFrontier(odom_pos_);
  visualization_->drawFrontier(planner_manager_->plan_data_.frontiers_);
}

bool TopoReplanFSM::callSearchAndOptimization() {}

bool TopoReplanFSM::callTopologicalTraj(int step) {
  bool plan_success;

  if (step == 1) plan_success = planner_manager_->planGlobalTraj(start_pt_);

  replan_time_.push_back(0.0);
  auto t1 = ros::Time::now();
  plan_success = planner_manager_->topoReplan(collide_);
  replan_time_[replan_time_.size() - 1] += (ros::Time::now() - t1).toSec();

  if (plan_success) {
    if (!act_map_) {

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // planner_manager_->planYawExplore(start_yaw_, start_yaw_[0], false, 1);
      planner_manager_->planYaw(start_yaw_);
    } else {
      replan_time2_.push_back(0);
      auto t1 = ros::Time::now();
      planner_manager_->planYawActMap(start_yaw_);
      replan_time2_[replan_time2_.size() - 1] += (ros::Time::now() - t1).toSec();
    }

    LocalTrajData *locdat = &planner_manager_->local_data_;

    /* publish newest trajectory to server */

    /* publish traj */
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = locdat->start_time_;
    bspline.traj_id = locdat->traj_id_;

    Eigen::MatrixXd pos_pts = locdat->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = locdat->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = locdat->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = locdat->yaw_traj_.getKnotSpan();

    bspline_pub_.publish(bspline);

    /* visualize new trajectories */

    MidPlanData *plan_data = &planner_manager_->plan_data_;

    visualization_->drawPolynomialTraj(planner_manager_->global_data_.global_traj_, 0.05, Eigen::Vector4d(0, 0, 0, 1), 0);
    visualization_->drawBspline(locdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1), true, 0.15,
                                Eigen::Vector4d(1, 1, 0, 1), 99);
    visualization_->drawBsplinesPhase2(plan_data->topo_traj_pos1_, 0.08);

    // visualization_->drawBspline(locdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1),
    // false, 0.15,
    //                             Eigen::Vector4d(1.0, 1.0, 1.0, 1), 99, 99);
    // visualization_->drawBspline(plan_data->no_visib_traj_, 0.08, Eigen::Vector4d(0.0,
    // 1.0, 1.0, 0.8),
    //                             true, 0.15, Eigen::Vector4d(1.0, 1.0, 1.0, 1), 98, 98);

    // visualization_->drawTopoPathsPhase2(plan_data->topo_select_paths_, 0.05);

    visualization_->drawViewConstraint(plan_data->view_cons_);
    visualization_->drawYawTraj(locdat->position_traj_, locdat->yaw_traj_, plan_data->dt_yaw_);
    // visualization_->drawYawPath(locdat->position_traj_, plan_data->path_yaw_,
    // plan_data->dt_yaw_path_);

    // visualization_->drawBspline(plan_data->no_visib_traj_, 0.08,
    //                             Eigen::Vector4d(0.0, 0.0, 1.0, 0.8), false,
    //                             0.1, Eigen::Vector4d(), 98);

    // visualization_->drawTopoGraph(planner_manager_->topo_graphs_,
    // 0.15, 0.05, Eigen::Vector4d(1, 0, 0, 1),
    //                               Eigen::Vector4d(0, 1, 0, 1),
    //                               Eigen::Vector4d(0, 0, 1, 1));

    // visualization_->drawTopoPathsPhase1(plan_data->topo_paths_, 0.05);
    // visualization_->drawBsplinesPhase1(plan_data->topo_traj_pos1_,
    // 0.065);

    // // benchmark, calculate replan time min max mean
    // double mean1 = 0.0;
    // double mean2 = 0.0;
    // for (auto t : replan_time_)
    //   mean1 += t;
    // for (auto t : replan_time2_)
    //   mean2 += t;
    // mean1 /= replan_time_.size();
    // mean2 /= replan_time2_.size();
    // ROS_WARN("Replan number: %d, mean traj: %lf, mean yaw: %lf", replan_time_.size(), mean1, mean2);

    return true;
  } else {
    return false;
  }
}
// TopoReplanFSM::
} // namespace fast_planner
