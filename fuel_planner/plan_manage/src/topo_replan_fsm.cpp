
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
  nh.param("fsm/enable_viz", _enable_viz, false);

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
  visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

  planner_manager_->setGlobalWaypoints(global_wp);
  have_target_ = true;
  trigger_ = true;
} // namespace fast_planner

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
  static uint _replan_num = 0;

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
    if (have_target_ and trigger_) {
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    } else {
      if (ros::Time::now() > wait_pub_timer) {
        _wait_goal_pub.publish(std_msgs::Empty());
        wait_pub_timer = ros::Time::now() + ros::Duration(1);
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

    /* topo path finding and optimization */
    if (callTopologicalTraj(1)) {
      changeFSMExecState(EXEC_TRAJ, "FSM");
    } else {
      ROS_WARN_STREAM(_label << "Planning failed. Retrying...");
      ros::Duration(0.5).sleep();
    }

    break;
  }

  case EXEC_TRAJ: {
    /* determine if need to replan */
    GlobalTrajData *global_data = &planner_manager_->global_data_;
    ros::Time time_now = ros::Time::now();
    double cur_time_pos = (time_now - global_data->global_start_time_).toSec();

    if (cur_time_pos > global_data->global_duration_ - 0.01) {
      // если осталось двигаться по траектории 0.01 сек
      have_target_ = false;
      changeFSMExecState(WAIT_TARGET, "FSM");
    } else {
      LocalTrajData *local_traj = &planner_manager_->local_data_;
      cur_time_pos = (time_now - local_traj->start_time_).toSec();

      if (cur_time_pos > replan_time_threshold_) {
        if (!global_data->localTrajReachTarget(0.8)) {
          ROS_DEBUG("%s[%u]Replan: periodic call", _label, _replan_num);
          changeFSMExecState(REPLAN_TRAJ, "FSM");
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

    if (not callTopologicalTraj(2))
      ROS_DEBUG("%s[%u]Replan failed", _label, _replan_num); //< если возвращает false, то возможно путь был стерт

    changeFSMExecState(EXEC_TRAJ, "FSM");

    ++_replan_num;

    break;
  }
  }
}

void TopoReplanFSM::checkCollisionCallback(const ros::TimerEvent &e) {

  /* ---------- check goal safety ---------- */
  // if (have_target_)
  /* if (false) {
    LocalTrajData &local_traj = planner_manager_->local_data_;
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_
                      ? edt_env->evaluateCoarseEDT(target_point_, local_traj.duration_)
                      : edt_env->evaluateCoarseEDT(target_point_, -1.0);

    if (dist <= 0.3) {
      //try to find a max distance goal around
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

            dist = planner_manager_->pp_.dynamic_ ? edt_env->evaluateCoarseEDT(new_pt, local_traj.duration_)
                                                  : edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              //reset target_point_
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
  } */

  /* ---------- check trajectory ---------- */
  if (exec_state_ == EXEC_TRAJ) {
    double dist;
    collide_ = not planner_manager_->checkTrajCollision(dist); //< функция возвращает false если есть препядствие.
    if (collide_) {
      ROS_WARN("%sCurrent traj %0.2f m to collision", _label, dist);
      if (dist > 1.0) {
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        ROS_WARN_STREAM(_label << "Replan. Collision detected");
      } else {
        new_pub_.publish(std_msgs::Empty()); //< остановка
        have_target_ = false;
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

bool TopoReplanFSM::callTopologicalTraj(int step) {
  bool plan_success;

  if (step == 1) plan_success = planner_manager_->planGlobalTraj(start_pt_); //< здесь есть расчет локальной траектории, зачем?

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

    LocalTrajData *local_traj = &planner_manager_->local_data_;

    /* publish newest trajectory to server */

    /* publish traj */
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = local_traj->start_time_;
    bspline.traj_id = local_traj->traj_id_;

    Eigen::MatrixXd pos_pts = local_traj->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = local_traj->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = local_traj->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = local_traj->yaw_traj_.getKnotSpan();

    bspline_pub_.publish(bspline);

    if (_enable_viz) visualization();

    return true;
  } else {
    return false;
  }
}

void TopoReplanFSM::visualization() {
  GlobalTrajData &global_data = planner_manager_->global_data_;
  MidPlanData &plan_data = planner_manager_->plan_data_;
  LocalTrajData *local_traj = &planner_manager_->local_data_;

  visualization_->drawPolynomialTraj(global_data.global_traj_, 0.05, Eigen::Vector4d(0, 0, 0, 1));
  visualization_->drawBspline(local_traj->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1), true, 0.15,
                              Eigen::Vector4d(1, 1, 0, 1));

  auto color1 = Eigen::Vector4d(223, 100, 153, 255) / 255.0;
  auto color2 = Eigen::Vector4d(62, 156, 190, 255) / 255.0;
  auto color3 = Eigen::Vector4d(72, 67, 70, 255) / 255.0;
  visualization_->drawTopoGraph(plan_data.topo_graph_, 0.08, 0.05, color1, color2, color3);

  // visualization_->drawBsplinesPhase2(plan_data.topo_traj_pos1_, 0.08);
  // visualization_->drawViewConstraint(plan_data.view_cons_);

  visualization_->drawYawTraj(local_traj->position_traj_, local_traj->yaw_traj_, plan_data.dt_yaw_);
}

} // namespace fast_planner
