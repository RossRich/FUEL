#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <eigen3/Eigen/Dense>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>

nav_msgs::Path point() {
  // Circle parameters
  nav_msgs::Path waypoints;
  geometry_msgs::PoseStamped pt;
  pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  double h = 1.0;
  double scale = 7.0;

  pt.pose.position.y = scale * 0.0;
  pt.pose.position.x = scale * 2.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 0.0;
  pt.pose.position.x = scale * 4.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 0.25;
  pt.pose.position.x = scale * 5.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 0.5;
  pt.pose.position.x = scale * 5.3;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 0.75;
  pt.pose.position.x = scale * 5.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 1.0;
  pt.pose.position.x = scale * 4.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 1.0;
  pt.pose.position.x = scale * 2.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = scale * 1.0;
  pt.pose.position.x = scale * 0.0;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);
  // Return
  return waypoints;
}

// Circle trajectory
nav_msgs::Path circle() {
  double h = 1.0;
  double scale = 5.0;
  nav_msgs::Path waypoints;
  geometry_msgs::PoseStamped pt;
  pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  pt.pose.position.y = -1.2 * scale;
  pt.pose.position.x = 2.5 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = -2.4 * scale;
  pt.pose.position.x = 5.0 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);
  pt.pose.position.y = 0.0 * scale;
  pt.pose.position.x = 5.0 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = -1.2 * scale;
  pt.pose.position.x = 2.5 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = -2.4 * scale;
  pt.pose.position.x = 0. * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);
  pt.pose.position.y = 0.0 * scale;
  pt.pose.position.x = 0.0 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = -1.2 * scale;
  pt.pose.position.x = 2.5 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = -2.4 * scale;
  pt.pose.position.x = 5.0 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);
  pt.pose.position.y = 0.0 * scale;
  pt.pose.position.x = 5.0 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = -1.2 * scale;
  pt.pose.position.x = 2.5 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  pt.pose.position.y = -2.4 * scale;
  pt.pose.position.x = 0. * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);
  pt.pose.position.y = 0.0 * scale;
  pt.pose.position.x = 0.0 * scale;
  pt.pose.position.z = h;
  waypoints.poses.push_back(pt);

  // Return
  return waypoints;
}

/*
 * cоздает регулярную сетку точек в заданной области
 * @note цент в середине заданной области
 * @param offset: отступ от краев огранич области
 * @param dist: расстояние между точками
 */
nav_msgs::Path dense(float src_size_x, float src_size_y, float offset, float dist, float rotation = 0.0) {
  float radians = rotation * M_PIf32 / 180.0f;
  float max_size = std::max(src_size_x, src_size_y);

  float origin_x = -max_size / 2.f;
  float origin_y = -max_size / 2.f;

  float size_x = max_size - offset * 2;
  float size_y = max_size - offset * 2;

  float reso = dist;

  Eigen::Vector3f trs(origin_x, origin_y, 0);
  auto rot_mat = Eigen::AngleAxisf(radians, Eigen::Vector3f(0, 0, 1)).matrix();

  nav_msgs::Path waypoints;
  geometry_msgs::PoseStamped pt;
  Eigen::Vector3f point_on_new_origin;
  pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  std::vector<geometry_msgs::PoseStamped> tmp_line;

  uint x_points_num = roundf(size_x / reso) + 1;
  uint y_points_num = roundf(size_y / reso) + 1;
  float new_x = 0.f;
  float new_y = 0.f;
  bool dir = true;

  auto constraint = [=](float p, float max, float min) { return p > max ? max - offset : p < min ? min + offset : p; };

  for (size_t xx = 0; xx < x_points_num; xx++) {
    tmp_line.clear();
    for (size_t yy = 0; yy < y_points_num; yy++) {
      //< новые точки
      new_x = xx * reso + offset;
      new_y = yy * reso + offset;

      //< двигаем точку в новое начало координат
      point_on_new_origin = Eigen::Vector3f(new_x, new_y, 1) + trs;
      point_on_new_origin = rot_mat * point_on_new_origin;

      if (constraint(point_on_new_origin.x(), src_size_x / 2.0, -src_size_x / 2.0) != point_on_new_origin.x()) continue;
      if (constraint(point_on_new_origin.y(), src_size_y / 2.0, -src_size_y / 2.0) != point_on_new_origin.y()) continue;

      pt.pose.position.x = point_on_new_origin.x();
      pt.pose.position.y = point_on_new_origin.y();
      pt.pose.position.z = 1;

      tmp_line.push_back(pt);
    }
    if (dir)
      waypoints.poses.insert(waypoints.poses.end(), tmp_line.begin(), tmp_line.end());
    else
      waypoints.poses.insert(waypoints.poses.end(), tmp_line.rbegin(), tmp_line.rend());

    dir = !dir;
  }
  return waypoints;
}

// Figure 8 trajectory
nav_msgs::Path eight() {
  // Circle parameters
  double offset_x = 0.0;
  double offset_y = 0.0;
  double r = 10.0;
  double h = 2.0;
  nav_msgs::Path waypoints;
  geometry_msgs::PoseStamped pt;
  pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  for (int i = 0; i < 1; ++i) {
    // First loop
    pt.pose.position.x = r + offset_x;
    pt.pose.position.y = -r + offset_y;
    pt.pose.position.z = h / 2;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 2 + offset_x * 2;
    pt.pose.position.y = 0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 3 + offset_x * 3;
    pt.pose.position.y = r;
    pt.pose.position.z = h / 2;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 4 + offset_x * 4;
    pt.pose.position.y = 0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 3 + offset_x * 3;
    pt.pose.position.y = -r;
    pt.pose.position.z = h / 2;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 2 + offset_x * 2;
    pt.pose.position.y = 0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r + offset_x * 2;
    pt.pose.position.y = r;
    pt.pose.position.z = h / 2;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = 0 + offset_x;
    pt.pose.position.y = 0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);
    // Second loop
    pt.pose.position.x = r + offset_x;
    pt.pose.position.y = -r;
    pt.pose.position.z = h / 2 * 3;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 2 + offset_x * 2;
    pt.pose.position.y = 0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 3 + offset_x * 3;
    pt.pose.position.y = r;
    pt.pose.position.z = h / 2 * 3;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 4 + offset_x * 4;
    pt.pose.position.y = 0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 3 + offset_x * 3;
    pt.pose.position.y = -r;
    pt.pose.position.z = h / 2 * 3;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r * 2 + offset_x * 2;
    pt.pose.position.y = 0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = r + offset_x;
    pt.pose.position.y = r + offset_y;
    pt.pose.position.z = h / 2 * 3;
    waypoints.poses.push_back(pt);
    pt.pose.position.x = 0;
    pt.pose.position.y = 0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);
  }
  return waypoints;
}
#endif