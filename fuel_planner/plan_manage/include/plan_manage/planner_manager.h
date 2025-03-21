#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline/non_uniform_bspline.h>
#include <bspline_opt/bspline_optimizer.h>

#include <path_searching/astar2.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/topo_prm.h>

#include <plan_env/edt_environment.h>

#include <active_perception/frontier_finder.h>
#include <active_perception/heading_planner.h>

#include <plan_manage/plan_container.hpp>
#include <traj_utils/planning_visualization.h>

#include <ros/ros.h>

namespace fast_planner {
// Fast Planner Manager
// Key algorithms of mapping and planning are called

using vector3d_t = Eigen::Vector3d;
using point3d_t = vector3d_t;
using points3d_t = std::vector<point3d_t>;
const Eigen::IOFormat vector3d_fmt(2, Eigen::DontAlign, ", ");

class FastPlannerManager {
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();

  /* main planning interface */
  bool kinodynamicReplan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                         const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, double time_lb = -1);

  /**
   * Создать траекторию по заданным точкам. 
   * Замедление (увеличение времени) в начальной и в конечной точках.
   *
   * @param points_traj набор контрольных точек
   * @param [out] poli_traj полиномиальная траектория
   * @return true если успех
   */
  bool create_flat_poli_traj(const points3d_t &points_traj, PolynomialTraj &poli_traj);

  /**
   * Построить траекторию по точкам
   * @warning необходимо 3 точки
   * @param tour вектор контрольных точек (не менее 3х)
   * @param cur_vel текущая скорость
   * @param cur_acc текущее ускорение
   * @param time_ld коэффициент для оптимизации пути по времени
   */
  void planExploreTraj(const vector<Eigen::Vector3d> &tour, const Eigen::Vector3d &cur_vel, const Eigen::Vector3d &cur_acc,
                       double time_lb = -1);
  bool planGlobalTraj(const Eigen::Vector3d &start_pos);
  bool planGlobalTraj2(const Eigen::Vector3d &start_pos);
  bool planGlobalTraj3(const vector3d_t &start_pos, const Eigen::Quaterniond &orientation);
  bool topoReplanLocalTraj(const ros::Time &time_now, const bool is_collide);
  bool refine_local_traj(const ros::Time &time_now, bool is_collide);

  /**
   * Работа с троекторией
   * @warning trag меняется. Сохранить если нужна старая траектория
   *
   * @param[in,out] traj Траектория для оптимизации
   * @param time_now Начало перестраения траектории
   * @param is_collide Обнаружено препядствие?
   * @return флаг успеха
   */
  bool topoReplanTraj(NonUniformBspline &traj, const ros::Time &time_now, const bool is_collide);

  void planYaw(const Eigen::Vector3d &start_yaw);
  void planYawExplore(const Eigen::Vector3d &start_yaw, const double &end_yaw, bool lookfwd, const double &relax_time);

  void initPlanModules(ros::NodeHandle &nh);
  void setGlobalWaypoints(vector<Eigen::Vector3d> &waypoints);

  bool fixPointInCollision(Eigen::Vector3d &point_in_collision);
  bool fixPointInCollision2(const Eigen::Quaterniond &orientation, point3d_t &point_in_collision);
  bool checkTrajCollision(double &distance);
  bool checkAgentCollision(int agent_id);
  void findCollisionRange(vector<Eigen::Vector3d> &colli_start, vector<Eigen::Vector3d> &colli_end,
                          vector<Eigen::Vector3d> &start_pts, vector<Eigen::Vector3d> &end_pts);
  void calcNextYaw(const double &last_yaw, double &yaw);

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  unique_ptr<Astar> path_finder_;
  unique_ptr<TopologyPRM> topo_prm_;
  std::unique_ptr<PlanningVisualization> _visualisation;

  AgentsData agents_data;

private:
  /* main planning algorithms & modules */
  shared_ptr<SDFMap> sdf_map_;

  unique_ptr<KinodynamicAstar> kino_path_finder_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;
  const char *_label = "[planner] ";

  void updateTrajInfo();

  // topology guided optimization

  void optimizeTopoBspline(double start_t, double duration, vector<Eigen::Vector3d> guide_path, int traj_id);

  /**
   * Определить локальную траекторию
   *
   * @param start_t Начало локальной траектории на глобальной (время)
   * @param[out] dt Время между контрольными точками на локальной траектории
   * @param[out] duration Общее время пути по локальной траектории
   * @return Крнтрольные точки локальной траектории (x,y,z)
   */
  Eigen::MatrixXd paramLocalTraj(double start_t, double &dt, double &duration);
  Eigen::MatrixXd reparamLocalTraj(const double &start_t, const double &duration, const double &dt);

  void selectBestTraj(NonUniformBspline &traj);
  void refineTraj(NonUniformBspline &best_traj);
  void reparamBspline(NonUniformBspline &bspline, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc);

  // Heading planning

  // !SECTION stable

  // SECTION developing

public:
  typedef shared_ptr<FastPlannerManager> Ptr;

  void planYawActMap(const Eigen::Vector3d &start_yaw);
  void test();
  void searchFrontier(const Eigen::Vector3d &p);
  bool findTopoPath();
  bool planLocaTraj(double start_time, const ros::Time &time_now);

private:
  unique_ptr<FrontierFinder> frontier_finder_;
  unique_ptr<HeadingPlanner> heading_planner_;
  unique_ptr<VisibilityUtil> visib_util_;

  // Benchmark method, local exploration
public:
  void select_final_goal(const Eigen::Vector3d &start, Eigen::Vector3d &goal);
  bool localExplore(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt);
  // scan_obst_slice(const Eigen::Vector3d &start, float radius,);

  // !SECTION
};
} // namespace fast_planner

#endif