#include "f110_opp_detect_node.hpp"

// ref:
// https://answers.ros.org/question/258425/converting-python-tf-code-to-c/
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
geometry_msgs::Quaternion toQuaternion(double yaw) {
  double roll = 0.0;
  double pitch = 0.0;
  geometry_msgs::Quaternion q;

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

double toEulerAngle(geometry_msgs::Quaternion q) {
  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return yaw;
}

std::vector<int> world_to_map(double x, double y) {
  double scale = loaded_map.info.resolution;
  double angle = toEulerAngle(loaded_map.info.origin.orientation);

  double trans_x = loaded_map.info.origin.position.x;
  double trans_y = loaded_map.info.origin.position.y;
  double world_x = x;
  double world_y = y;
  double dx = world_x - trans_x;
  double dy = world_y - trans_y;

  // rot matrxi [ [c, -s], [s, c] ]
  double c = cos(angle);
  double s = sin(angle);

  // map_c = rot*((world - trans) / float(scale))
  //       = [ [c,-s]  * [dx   / scale
  //           [s, c]]    dy]
  double map_idx_y = (c*dx - s*dy) / scale;
  double map_idx_x = (s*dx + c*dy) / scale;

  std::vector<int> res;
  res.push_back((int)map_idx_x);
  res.push_back((int)map_idx_y);
  return res;
}

void visualize_opp(std::vector<std::vector<double>> opp_points) {
  pid_t tid = syscall(SYS_gettid);
  std::cout << "thread in " << "visualize_opp is: " << tid << "\n";

  visualization_msgs::Marker points_list;
  points_list.header.frame_id = "map";
  points_list.header.stamp = ros::Time::now();
  points_list.action = visualization_msgs::Marker::ADD;
  points_list.pose.orientation.w = 1.0;

  points_list.type = visualization_msgs::Marker::POINTS;
  points_list.id = 0; // opt = point;
  points_list.ns = "point_marker";

  points_list.scale.x = points_list.scale.y = 0.1;
  points_list.color.b = 0.0;
  points_list.color.r = 1.0;
  points_list.color.a = 1.0;

  for(int i = 0; i<opp_points.size(); i ++){
    geometry_msgs::Point p;
    p.x = opp_points[i][0];
    p.y = opp_points[i][1];
    p.z = 0.0;
    points_list.points.push_back(p);
  }

  pub_opp_vis.publish(points_list);
}

void publish_filtered_lidar(sensor_msgs::LaserScan filtered_lidar) {
  pub_filtered_lidar.publish(filtered_lidar);
}
