#include "utils.hpp"

ScanSimulator2D scan_simulator;
double angle_min;
double angle_max;
double angle_inc;
double scan_distance_to_base_link_CoG;
std::vector<double> scan_angles;

void init_sim() {
  // params
  int scan_beams = 1080;
  double scan_fov = 4.7;
  double scan_std_dev = 0.01;
  angle_min = -scan_fov / 2;
  angle_max = +scan_fov / 2;
  angle_inc = scan_fov / scan_beams;
  scan_angles = std::vector<double>(scan_beams);
  for(int i = 0; i < scan_beams; i++) {
    double angle = -scan_fov/2.0 + i*angle_inc;
    scan_angles[i] = angle;
  }
  scan_distance_to_base_link_CoG = 0.275 - 0.3302 / 2;

  // init lidar simulator
  scan_simulator = ScanSimulator2D(scan_beams, scan_fov, scan_std_dev);

  // get map and feed into scan simulator
  ros::NodeHandle nh;
  if (!requestMap(nh))
    exit(-1);
  std::vector<double> map_vector;
  for (int i = 0; i <  loaded_map.info.height * loaded_map.info.width; i++)
    map_vector.push_back(loaded_map.data[i]);
  Pose2D map_origin;
  map_origin.x = loaded_map.info.origin.position.x;
  map_origin.y = loaded_map.info.origin.position.y;
  double free_threshold = 0.8;
  scan_simulator.set_map(map_vector,
    loaded_map.info.height,
    loaded_map.info.width,
    loaded_map.info.resolution,
    map_origin,
    free_threshold
  );
}

sensor_msgs::LaserScan make_lidar_msg(std::vector<double> current_scan) {
  // make a scan msg
  sensor_msgs::LaserScan scan;
  scan.header.stamp = ros::Time::now();
  scan.header.frame_id = "ego_racecar/laser";
  scan.angle_min = angle_min;
  scan.angle_max = angle_max;
  scan.angle_increment = angle_inc;
  scan.range_min = 0.0;
  scan.range_max = 30.0;
  scan.ranges.resize(current_scan.size());
  for(int i = 0; i < current_scan.size(); i++) {
    scan.ranges[i] = current_scan[i];
  }
  return scan;
}

nav_msgs::Odometry make_odom_msg(Pose2D ego_state) {
  // make a odom msg
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "map";
  geometry_msgs::Quaternion q_msg;
  q_msg.w = cos(ego_state.theta * 0.5);
  q_msg.z = sin(ego_state.theta * 0.5);
  odom.pose.pose.position.x = ego_state.x;
  odom.pose.pose.position.y = ego_state.y;
  odom.pose.pose.orientation = q_msg;
  return odom;
}

std::vector<double> get_scan(Pose2D ego_state, Pose2D opp_state) {
  // init ego state
  Pose2D ego_scan_pose;
  ego_scan_pose.x = ego_state.x + scan_distance_to_base_link_CoG * std::cos(ego_state.theta);
  ego_scan_pose.y = ego_state.y + scan_distance_to_base_link_CoG * std::sin(ego_state.theta);
  ego_scan_pose.theta = ego_state.theta;

  // get scan simulation
  std::vector<double> current_scan = scan_simulator.scan(ego_scan_pose);

  // modify the scan according to opp pose
  ray_cast_opponents(current_scan, scan_angles, ego_scan_pose, opp_state);

  return current_scan;
}

/*
void transform_from_relative_to_absolute_pos(Eigen::VectorXd & pos, Eigen::VectorXd & state) {
  double s = state[0];
  if (s>87.1) s -= 87.1;

  double ey = state[1];
  double ephi = state[2];

  int box_nb = 12;
  for (int i = 0; i < 12; i++) {
    if (s>=LUT[i][8]&&s<LUT[i+1][8]) {
      box_nb = i;
      break;
    }
  }

  double s0 = LUT[box_nb][8];
  double x0 = LUT[box_nb][4];
  double y0 = LUT[box_nb][5];
  double x1 = LUT[box_nb][6];
  double y1 = LUT[box_nb][7];

  if (box_nb==0||box_nb==4||box_nb==6||box_nb==8||box_nb==10||box_nb==12) {
    // the case for LINE-type parts
    double bx = x1-x0;
    double by = y1-y0;
    double b_norm = sqrt(bx*bx + by*by);

    double px = bx/b_norm * (s-s0) + x0;
    double py = by/b_norm * (s-s0) + y0;

    double ey_theta = atan2(by, bx) + PI/2;
    pos(0) = px + ey * cos(ey_theta);
    pos(1) = py + ey * sin(ey_theta);
    pos(2) = ephi + atan2(by,bx);
  } else {
    // the case for CIRCLE-type parts
    if (box_nb==2||box_nb==7) {
      // counter-clockwise cases
      double r0x = (x0+x1)/2;
      double r0y = (y0+y1)/2;
      double theta0 = atan2(y0-r0y, x0-r0x);
      double theta1 = (s - s0)/2.5 + theta0;

      pos(0) = (2.5-ey) * cos(theta1) + r0x;
      pos(1) = (2.5-ey) * sin(theta1) + r0y;
      pos(2) = ephi + (theta1+PI/2);;
    } else {
      // clockwise cases
      double r0x, r0y;
      if (box_nb==5) {
        r0x = x0;
        r0y = y1;
      } else
      if (box_nb==11) {
        r0x = x1;
        r0y = y0;
      }
      else {
        r0x = (x0+x1)/2;
        r0y = (y0+y1)/2;
      }
      double theta0 = atan2(y0-r0y, x0-r0x);
      double theta1 = -(s - s0)/2.5 + theta0;

      pos(0) = (2.5+ey) * cos(theta1) + r0x;
      pos(1) = (2.5+ey) * sin(theta1) + r0y;
      pos(2) = ephi + (theta1-PI/2);
    }
  }
}
*/
void ray_cast_opponents(std::vector<double> &scan, std::vector<double> &scan_angles, const Pose2D &scan_pose, Pose2D &op_pose) {
    double car_width = 0.31;
    double car_length = 0.58;

    double x = op_pose.x;
    double y = op_pose.y;
    double theta = op_pose.theta;

    Eigen::Vector2d diff_x {std::cos(theta), std::sin(theta)};
    Eigen::Vector2d diff_y {-std::sin(theta), std::cos(theta)};
    diff_x = (car_length/2) * diff_x;
    diff_y = (car_width/2) * diff_y;

    auto c1 = diff_x - diff_y;
    auto c2 = diff_x + diff_y;
    auto c3 = diff_y - diff_x;
    auto c4 = -diff_x - diff_y;

    Eigen::Vector2d corner1 {x+c1(0), y+c1(1)};
    Eigen::Vector2d corner2 {x+c2(0), y+c2(1)};
    Eigen::Vector2d corner3 {x+c3(0), y+c3(1)};
    Eigen::Vector2d corner4 {x+c4(0), y+c4(1)};

    std::vector<Eigen::Vector2d> bounding_boxes {corner1, corner2, corner3, corner4, corner1};

    // modify scan
    for (size_t i=0; i < scan_angles.size(); i++) {
        for (size_t j=0; j<4; j++) {
            double range = get_range(scan_pose, scan_pose.theta + scan_angles[i], bounding_boxes[j], bounding_boxes[j+1]);
            if (range < scan[i]) {
                scan[i] = range;
            }
        }
    }
}

double cross(Eigen::Vector2d v1, Eigen::Vector2d v2) {
    return v1(0) * v2(1) - v1(1) * v2(0);
}
bool are_collinear(Eigen::Vector2d pt_a, Eigen::Vector2d pt_b, Eigen::Vector2d pt_c) {
    double tol = 1e-8;
    auto ba = pt_b - pt_a;
    auto ca = pt_a - pt_c;
    bool col = (std::fabs(cross(ba, ca)) < tol);
    return col;
}

double get_range(const Pose2D &pose, double beam_theta, Eigen::Vector2d la, Eigen::Vector2d lb) {
    Eigen::Vector2d o {pose.x, pose.y};
    Eigen::Vector2d v1 = o - la;
    Eigen::Vector2d v2 = lb - la;
    Eigen::Vector2d v3 {std::cos(beam_theta + PI/2.0), std::sin(beam_theta + PI/2.0)};

    double denom = v2.dot(v3);

    double x = INFINITY;

    if (std::fabs(denom) > 0.0) {
        double d1 = cross(v2, v1) / denom;
        double d2 = v1.dot(v3) / denom;
        if (d1 >= 0.0 && d2 >= 0.0 && d2 <= 1.0) {
            x = d1;
        }
    } else if (are_collinear(o, la, lb)) {
        double dist_a = (la - o).norm();
        double dist_b = (lb - o).norm();
        x = std::min(dist_a, dist_b);
    }
    return x;
}
