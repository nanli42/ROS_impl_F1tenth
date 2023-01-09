#include "utils.hpp"

std::vector<std::vector<bool>> grid;
nav_msgs::OccupancyGrid loaded_map;
sensor_msgs::LaserScan newestLidarMsg;
sensor_msgs::LaserScan filteredLidarMsg;

double init_time;
double last_lidar_receiving_time = 0;
std::ofstream file_;
std::ofstream file_trace_;
ros::Publisher pub_opp_vis;
ros::Publisher pub_filtered_lidar;
ros::Publisher pub_opp_odom;

int main(int argc, char **argv) {
  set_main_thread_sched_priority(0, 98, SCHED_FIFO);

  file_.open("/home/nvidia/test_by_Nan_Li/timing/test_WCET_op.txt");
  //file_.open("/home/nan/workspace/project/run.f1tenth/timing/test_WCET_op.txt");

  // init a ros handler
  ros::init(argc, argv, "test_WCET");

  ros::WallTimerEvent t_virtual;
  auto t_start = std::chrono::high_resolution_clock::now();
  auto t_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> dt_ms;

  sensor_msgs::LaserScan scan;
  nav_msgs::Odometry odom;
  sensor_msgs::LaserScan::Ptr scan_ptr = boost::shared_ptr<sensor_msgs::LaserScan>(&scan);
  nav_msgs::Odometry::Ptr odom_ptr = boost::shared_ptr<nav_msgs::Odometry>(&odom);

  init_sim();

  Eigen::VectorXd ego_x_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd ego_ex_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd opp_x_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd opp_ex_ = Eigen::VectorXd::Zero(3);

  default_random_engine generator;
  uniform_real_distribution<double> distrib_ego_s(0, 87.0);
  uniform_real_distribution<double> distrib_ego_ey(-1.3, +1.3);
  uniform_real_distribution<double> distrib_ego_ephi(-0.75, +0.75);
  uniform_real_distribution<double> distrib_opp_d_s(1.0, 20.0);
  uniform_real_distribution<double> distrib_opp_ey(-1.3, +1.3);
  uniform_real_distribution<double> distrib_opp_ephi(-0.75, +0.75);

  double mean_time = 0.0;
  double max_time = -1e+6;
  for (int i = 0; i < 100000; i++ ) {
    ego_ex_(0) = distrib_ego_s(generator); // s
    ego_ex_(1) = distrib_ego_ey(generator); // ey
    ego_ex_(2) = distrib_ego_ephi(generator); // ephi
    opp_ex_(0) = ego_ex_(0) + distrib_opp_d_s(generator); // s
    opp_ex_(1) = distrib_opp_ey(generator); // ey
    opp_ex_(2) = distrib_opp_ephi(generator); // ephi
    transform_from_relative_to_absolute_pos(ego_x_, ego_ex_);
    transform_from_relative_to_absolute_pos(opp_x_, opp_ex_);

    //std::cout << "ego: " << ego_x_(0) << ", " << ego_x_(1) << ", " << ego_x_(2) << std::endl;
    //std::cout << "opp: " << opp_x_(0) << ", " << opp_x_(1) << ", " << opp_x_(2) << std::endl;

    Pose2D ego_state {ego_x_(0), ego_x_(1), ego_x_(2)};
    Pose2D opp_state {opp_x_(0), opp_x_(1), opp_x_(2)};
    std::vector<double> current_scan = get_scan(ego_state, opp_state);

    scan = make_lidar_msg(current_scan);
    odom = make_odom_msg(ego_state);

    lidarLogger(scan_ptr);
    poseLogger(odom_ptr);

    t_start = std::chrono::high_resolution_clock::now();
    oppDetector(t_virtual);
    t_end = std::chrono::high_resolution_clock::now();
    dt_ms = t_end - t_start;

    file_ << dt_ms.count() << std::endl;
    if (i>0)
      mean_time = (mean_time * i + dt_ms.count()) / (i + 1);
    else
      mean_time = dt_ms.count();
    if (dt_ms.count()>max_time) max_time = dt_ms.count();

    if (i%1000==0) std::cout << "iter: " << i << ", max_time [ms] " << max_time << "\n" << std::endl;
  }
  file_ << "---" << std::endl;
  file_ << mean_time << std::endl;
  file_ << max_time << std::endl;

  return 0;
}
