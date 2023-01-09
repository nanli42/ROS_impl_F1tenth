#include "f110_opp_detect_node.hpp"

#define PI 3.1415926
#define TEST_WCET 0

double currentX = 0.275-0.3302/2;
double currentY = 1.14;
double currentYAW = 0.0;

extern std::ofstream file_;
extern std::ofstream file_trace_;

extern std::chrono::time_point<std::chrono::high_resolution_clock> chrono_init_time;
extern sensor_msgs::LaserScan filteredLidarMsg;

bool calibration = false;

void lidarLogger(const sensor_msgs::LaserScan::ConstPtr& msg) {
  //ROS_INFO("in lidarLogger... %f - delay %f", ros::Time::now().toSec() - last_lidar_receiving_time, ros::Time::now().toSec() - msg->header.stamp.toSec());
  last_lidar_receiving_time = ros::Time::now().toSec();
  newestLidarMsg = *msg;
}

void poseLogger(const nav_msgs::Odometry::ConstPtr& msg) {
  double laser_to_base_link = 0.275 - 0.3302/2;

  currentX = msg->pose.pose.position.x;
  currentY = msg->pose.pose.position.y;
  currentYAW = toEulerAngle(msg->pose.pose.orientation);

  currentX += laser_to_base_link*cos(currentYAW);
  currentY += laser_to_base_link*sin(currentYAW);
}

double calc_dist(std::vector<double> p1, std::vector<double> p2) {
  double dx = p1[0] - p2[0];
  double dy = p1[1] - p2[1];
  return sqrt(dx*dx+dy*dy);
}

std::vector<double> oppFindCenter(std::vector<std::vector<double>> opp_points, int min_range_idx) {
  pid_t tid = syscall(SYS_gettid);
  //std::cout << "thread in " << "oppFindCenter is: " << tid << "\n";

  std::vector<double> point_center;
  std::vector<int> vertex;

  int end_idx = opp_points.size() - 1;
  if ( min_range_idx == 0 || min_range_idx == end_idx ) {
    vertex.push_back(0);
    vertex.push_back(end_idx);
  } else if (calc_dist(opp_points[0], opp_points[end_idx]) < 0.58) {
    vertex.push_back(0);
    vertex.push_back(end_idx);
  } else {
    vertex.push_back(0);
    vertex.push_back(min_range_idx);
    vertex.push_back(end_idx);
  }

  // if L-shape
  if (vertex.size()==3) {
    std::vector<double> point_A = opp_points[vertex[0]];
    std::vector<double> point_B = opp_points[vertex[1]];
    std::vector<double> point_C = opp_points[vertex[2]];

    std::vector<double> long_edge_vector;

    double dist_AB = calc_dist(point_A, point_B);
    double dist_BC = calc_dist(point_B, point_C);
    double dist_CA = calc_dist(point_C, point_A);
    double dist_max;
    if (dist_AB>dist_BC && dist_AB>dist_CA) dist_max = dist_AB;
    if (dist_BC>dist_AB && dist_BC>dist_CA) dist_max = dist_BC;
    if (dist_CA>dist_BC && dist_CA>dist_AB) dist_max = dist_CA;

    if (dist_AB>0.8 || dist_BC>0.8 || dist_CA>0.8 || dist_AB<0.2 || dist_BC<0.2 || dist_CA<0.2 ) {
      ROS_ERROR("not a correct opponent shape! (1)");
      return point_center;
    }
    else if (dist_AB==dist_max) {
      point_center.push_back(point_A[0] + (point_B[0]-point_A[0])/2);
      point_center.push_back(point_A[1] + (point_B[1]-point_A[1])/2);
      if (dist_BC>dist_CA) {
        long_edge_vector.push_back(point_B[0] - point_C[0]);
        long_edge_vector.push_back(point_B[1] - point_C[1]);
      }
      else {
        long_edge_vector.push_back(point_A[0] - point_C[0]);
        long_edge_vector.push_back(point_A[1] - point_C[1]);
      }
    }
    else if (dist_BC==dist_max) {
      point_center.push_back(point_B[0] + (point_C[0]-point_B[0])/2);
      point_center.push_back(point_B[1] + (point_C[1]-point_B[1])/2);
      if (dist_AB>dist_CA) {
        long_edge_vector.push_back(point_B[0] - point_A[0]);
        long_edge_vector.push_back(point_B[1] - point_A[1]);
      }
      else {
        long_edge_vector.push_back(point_C[0] - point_A[0]);
        long_edge_vector.push_back(point_C[1] - point_A[1]);
      }
    }
    else if (dist_CA==dist_max) {
      point_center.push_back(point_C[0] + (point_A[0]-point_C[0])/2);
      point_center.push_back(point_C[1] + (point_A[1]-point_C[1])/2);
      if (dist_BC>dist_AB) {
        long_edge_vector.push_back(point_C[0] - point_B[0]);
        long_edge_vector.push_back(point_C[1] - point_B[1]);
      }
      else {
        long_edge_vector.push_back(point_A[0] - point_B[0]);
        long_edge_vector.push_back(point_A[1] - point_B[1]);
      }
    }
    double norm = sqrt(long_edge_vector[0]*long_edge_vector[0] + long_edge_vector[1]*long_edge_vector[1]);
    long_edge_vector[0] = long_edge_vector[0] / norm;
    long_edge_vector[1] = long_edge_vector[1] / norm;
    std::vector<double> horizon_vector = {1, 0};
    double dot = long_edge_vector[0];
    if (dot<-1) dot = -1;
    else if (dot>1) dot = 1;
    double yaw_center = acos(dot);
    double yaw1, yaw2;
    if (long_edge_vector[1]<0) {
      yaw1 = -yaw_center;
      yaw2 = PI - yaw_center;
    }
    else {
      yaw1 = yaw_center;
      yaw2 = -(PI - yaw_center);
    }

    double opp_ego_angle = currentYAW;
    double d1 = abs(yaw1 - opp_ego_angle);
    if (d1 > PI) d1 = 2*PI - d1;
    double d2 = abs(yaw2 - opp_ego_angle);
    if (d2 > PI) d2 = 2*PI - d2;
    if (d1 < d2) yaw_center = yaw1;
    else  yaw_center = yaw2;
    if (yaw_center > PI) yaw_center = -(2*PI-yaw_center);
    if (yaw_center < -PI) yaw_center = +(2*PI+yaw_center);
    point_center.push_back(yaw_center);
  }
  // if I-shape
  else if (vertex.size()==2) {
    std::vector<double> point_A = opp_points[vertex[0]];
    std::vector<double> point_B = opp_points[vertex[1]];

    double dist_AB = calc_dist(point_A, point_B);
    if (dist_AB>0.8 || dist_AB<0.2) {
      ROS_ERROR("not a correct opponent shape! (2)");
      return point_center;
    }
    else {
      double unit_vector_x = (point_A[0]-point_B[0])/dist_AB;
      double unit_vector_y = (point_A[1]-point_B[1])/dist_AB;
      double vector_to_center_x = unit_vector_y * 1;
      double vector_to_center_y = unit_vector_x * -1;
      double norm = sqrt(vector_to_center_x*vector_to_center_x + vector_to_center_y*vector_to_center_y);
      vector_to_center_x = vector_to_center_x / norm;
      vector_to_center_y = vector_to_center_y / norm;

      double dd;
      if (dist_AB>0.4) dd = 0.31/2;
      else dd = 0.58/2;

      double A_to_ego_x = point_A[0] - currentX;
      double A_to_ego_y = point_A[1] - currentY;
      if (A_to_ego_x*vector_to_center_x + A_to_ego_y*vector_to_center_y > 0) {
        point_center.push_back(point_A[0] + (point_B[0]-point_A[0])/2 + dd * vector_to_center_x);
        point_center.push_back(point_A[1] + (point_B[1]-point_A[1])/2 + dd * vector_to_center_y);
      } else {
        point_center.push_back(point_A[0] + (point_B[0]-point_A[0])/2 - dd * vector_to_center_x);
        point_center.push_back(point_A[1] + (point_B[1]-point_A[1])/2 - dd * vector_to_center_y);
      }
      double yaw_center;
      if (dist_AB>0.4) {
        yaw_center = currentYAW;
      } else {
        yaw_center = opp_points[min_range_idx][2];
      }

      if (yaw_center > PI) yaw_center = -(2*PI-yaw_center);
      if (yaw_center < -PI) yaw_center = +(2*PI+yaw_center);
      point_center.push_back(yaw_center);
    }

  } else {
    ROS_INFO("vertex.size() = %d", vertex.size());
    ROS_ERROR("not a correct opponent shape! (3)");
  }

  return point_center;
}

void oppDetector(const ros::WallTimerEvent& event) {
//void oppDetector(const sensor_msgs::LaserScan::ConstPtr& msg) {
//void oppDetector(const std_msgs::Bool::ConstPtr& msg) {
  auto t_start = std::chrono::high_resolution_clock::now();
  if (!TEST_WCET) {
    auto millis = std::chrono::duration_cast<std::chrono::microseconds>(t_start.time_since_epoch()).count();
    double t_ms = (millis % 1000000) / 1000.0;
    ROS_INFO("--- in oppDetector...: %7.3f [ms]", t_ms);

    //pid_t tid = syscall(SYS_gettid);
    //std::cout << "thread in " << "oppDetector is: " << tid << "\n";

    //ROS_INFO_STREAM("--- in oppDetector...  " << millis);
    double t1 = ros::Time::now().toSec() - init_time;
  }

  if (newestLidarMsg.ranges.empty()) return;
  sensor_msgs::LaserScan currentLidarMsg = newestLidarMsg;

  std::vector<std::vector<double>> opp_points;
  int margin = 4; // 4 pixels as index margin
  double min_range = 1e+6;
  int min_range_idx = -1;
  int opp_count = 0;
  for (int i = 0; i < (currentLidarMsg.ranges).size(); i++) {
    double yaw_ = currentYAW + currentLidarMsg.angle_min + i * currentLidarMsg.angle_increment;
    double x_ = currentX + cos(yaw_) * currentLidarMsg.ranges[i];
    double y_ = currentY + sin(yaw_) * currentLidarMsg.ranges[i];
    std::vector<int> map_idx = world_to_map(x_, y_);
    if ( (map_idx[0]-margin>0 && map_idx[0]+margin<loaded_map.info.height) &&
         (map_idx[1]-margin>0 && map_idx[1]+margin<loaded_map.info.width)
       ) {
         if ( grid[map_idx[0]][map_idx[1]]
           && grid[map_idx[0]-margin][map_idx[1]-margin]
           && grid[map_idx[0]-margin][map_idx[1]+margin]
           && grid[map_idx[0]+margin][map_idx[1]-margin]
           && grid[map_idx[0]+margin][map_idx[1]+margin]
         ) {
           std::vector<double> new_point;
           new_point.push_back(x_);
           new_point.push_back(y_);
           new_point.push_back(yaw_);
           opp_points.push_back(new_point);

           if (currentLidarMsg.ranges[i] < min_range) {
             min_range =  currentLidarMsg.ranges[i];
             min_range_idx = opp_count;
           }

           opp_count++;
           currentLidarMsg.ranges[i] = 0;
         }
    }
  }

  //ROS_INFO("opp exist: %d", !opp_points.empty());
  std::vector<double> point_center;
  if (!opp_points.empty()) {
    point_center = oppFindCenter(opp_points, min_range_idx);
    if (!point_center.empty()) {
      //ROS_INFO("POINT_CENTER:  %f, %f, %f", point_center[0], point_center[1], point_center[2]);
      opp_points.push_back(point_center);
    }
  }


  if (!TEST_WCET) {

  //ROS_INFO(" opp_detect - duration: %f", ros::Time::now().toSec() - init_time - t1);
  double t2 = ros::Time::now().toSec() - init_time;

  //exec_pf(currentLidarMsg);

  //ROS_INFO(" pf -duration: %f", ros::Time::now().toSec() - init_time - t2);
  //ROS_INFO("=== after oppDetector...  %f\n", ros::Time::now().toSec() - init_time);

  publish_filtered_lidar(currentLidarMsg);

  //============================================================================//
    auto end_time = std::chrono::high_resolution_clock::now();

    auto t_end_millis = std::chrono::duration_cast<std::chrono::microseconds>(end_time.time_since_epoch()).count();
    double t_end_ms = (t_end_millis % 1000000) / 1000.0;
    ROS_INFO("--- end of oppDetector...: %7.3f [ms]\n", t_end_ms);
    /*
    std_msgs::Bool msg;
    msg.data = true;
    pub_opp_complete.publish(msg);
    */
    /*
    using std::chrono::operator""ns;
    std::this_thread::sleep_until(t_start+100000000ns);
    */
    //auto exit_time = std::chrono::high_resolution_clock::now();
    file_ << currentLidarMsg.header.stamp.sec << currentLidarMsg.header.stamp.nsec << ", ";
    //file_ << std::chrono::duration_cast<std::chrono::nanoseconds>(t_start.time_since_epoch()).count() << ", ";
    //file_ << std::chrono::duration_cast<std::chrono::nanoseconds>(end_time.time_since_epoch()).count()  << ", ";
    std::chrono::duration<double, std::milli> dt_ms = end_time - t_start;
    file_ << dt_ms.count()  << std::endl;
    /*
    std::chrono::duration<double, std::milli> dt_ms2 = exit_time - t_start;
    file_ << ", " << std::chrono::duration_cast<std::chrono::nanoseconds>(exit_time.time_since_epoch()).count()  << ", ";
    file_ << dt_ms2.count()  << std::endl;
    */

    file_trace_ << t_start.time_since_epoch().count() << ", " << end_time.time_since_epoch().count() << std::endl;
  //============================================================================//

  // Publish odom
  nav_msgs::Odometry opp_odom;
  opp_odom.header.stamp = ros::Time::now();
  opp_odom.header.frame_id = "map";

  if (!opp_points.empty()) {
    visualize_opp(opp_points);
  }
  if (!point_center.empty()) {
    std::cout << point_center[0] << ", " << point_center[1] << ", " << point_center[2] << std::endl;

    opp_odom.pose.pose.position.x = point_center[0];
    opp_odom.pose.pose.position.y = point_center[1];
    geometry_msgs::Quaternion q_msg;
    q_msg.w = cos(0.5 * point_center[2]);
    q_msg.z = sin(0.5 * point_center[2]);
    opp_odom.pose.pose.orientation = q_msg;
    pub_opp_odom.publish(opp_odom);

  } else {
    opp_odom.pose.pose.position.x = 1e+6;
    pub_opp_odom.publish(opp_odom);
  }
  } else {
      filteredLidarMsg = currentLidarMsg;

      /*
      if (!point_center.empty())
        std::cout << "op result: " << point_center[0] << ", " << point_center[1] << ", " << point_center[2] << std::endl;
      else
        std::cout << "op result: " << "no opp" << std::endl;
      */
  }
}
