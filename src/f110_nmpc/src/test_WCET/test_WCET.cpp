#include "utils.hpp"

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

double qp_calc_period;
int who_am_i = -1; // 0 - ego, 1 - opp
ros::TimerEvent init_event;

Eigen::VectorXd ego_x_ = Eigen::VectorXd::Zero(8); //position of the vehicle (x,y,phi,v,omega)
Eigen::VectorXd ego_ex_ = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)

Eigen::VectorXd opp_x_ = Eigen::VectorXd::Zero(8); //position of the vehicle (x,y,phi,v,omega)
Eigen::VectorXd opp_ex_ = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)

Eigen::VectorXd ego_ex_snapshot_ = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)
Eigen::VectorXd opp_ex_snapshot_ = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)

ros::Subscriber sub_ego_odom;
ros::Subscriber sub_opp_odom;

ros::Publisher cmd_pub;
ros::Publisher traj_marker_pub;
ros::Publisher pos_marker_pub;
ros::Publisher odom_delta_pub;
ros::Publisher compensated_pose_pub;

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

ACADOvariables acadoVariables_opp;

ACADOvariables acadoVariables_LO;
ACADOworkspace acadoWorkspace_LO;
ACADOvariables acadoVariables_RO;
ACADOworkspace acadoWorkspace_RO;

double lr = 0.17145;
double lf = 0.3302 - 0.17145;

double KKT_update_cmd = 0.0;

double LO_iter_counter = 0;
double LO_time_counter = 0;
double LO_time_max = -1e+6;
double LO_time_min = +1e+6;
double RO_iter_counter = 0;
double RO_time_counter = 0;
double RO_time_max = -1e+6;
double RO_time_min = +1e+6;
double FO_iter_counter = 0;
double FO_time_counter = 0;
double FO_time_max = -1e+6;
double FO_time_min = +1e+6;

Eigen::VectorXd prev_cmd = Eigen::VectorXd::Zero(4); // speed, steering_angle, a, vd
Eigen::VectorXd curr_cmd = Eigen::VectorXd::Zero(4); // speed, steering_angle, a, vd
ros::Time ego_odom_time_stamp;

std::mutex qp_mutex;

int state_machine_current_mode;
int state_machine_previous_mode;
int time_count_L2F = 0;
int time_count_F2L = 0;
bool RESP;

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

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

int LO_iter_num;
int RO_iter_num;
int FW_iter_num;
double LO_kkt_val;
double RO_kkt_val;
double FW_kkt_val;
double LO_time;
double RO_time;
double FW_time;
double FO_time;
double PREP_time;
double AFTER_time;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

int main(int argc, char **argv) {
  set_main_thread_sched_priority(0, 98, SCHED_FIFO);

  std::ofstream file_;
  file_.open("/home/nvidia/test_by_Nan_Li/timing/test_WCET_nmpc.txt");
  //file_.open("/home/nan/workspace/project/run.f1tenth/timing/test_WCET_nmpc.txt");

  // init a ros handler
  ros::init(argc, argv, "test_WCET");

  ros::NodeHandle nh_ctrl;
  nh_ctrl.getParam("/qp_calc_period", qp_calc_period);
  nh_ctrl.getParam("/who_am_i", who_am_i);

  ros::TimerEvent t_virtual;
  auto t_start = std::chrono::high_resolution_clock::now();
  auto t_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> dt_ms;

  sensor_msgs::LaserScan scan;
  nav_msgs::Odometry odom;
  sensor_msgs::LaserScan::Ptr scan_ptr = boost::shared_ptr<sensor_msgs::LaserScan>(&scan);
  nav_msgs::Odometry::Ptr odom_ptr = boost::shared_ptr<nav_msgs::Odometry>(&odom);

  init_sim();

  //default_random_engine generator;
  std::mt19937 generator(0);
  uniform_real_distribution<double> distrib_ego_s(0, 87.0);
  uniform_real_distribution<double> distrib_ego_ey(-1.3, +1.3);
  uniform_real_distribution<double> distrib_ego_ephi(-0.75, +0.75);
  uniform_real_distribution<double> distrib_ego_vx(+0.05, +2);
  uniform_real_distribution<double> distrib_ego_vy(-0.5, +0.5);
  uniform_real_distribution<double> distrib_ego_omega(-4.0, +4.0);
  uniform_real_distribution<double> distrib_ego_delta(-0.41, +0.41);

  uniform_real_distribution<double> distrib_opp_d_s(1.0, 20.0);
  uniform_real_distribution<double> distrib_opp_ey(-1.3, +1.3);
  uniform_real_distribution<double> distrib_opp_ephi(-0.75, +0.75);
  uniform_real_distribution<double> distrib_opp_vx(+0.05, +1);
  uniform_real_distribution<double> distrib_opp_vy(-0.25, +0.25);
  uniform_real_distribution<double> distrib_opp_omega(-4.0, +4.0);
  uniform_real_distribution<double> distrib_opp_delta(-0.41, +0.41);

  double mean_time = 0.0;
  double max_time = -1e+6;

  double LO_mean_time = 0.0;
  double RO_mean_time = 0.0;
  double FW_mean_time = 0.0;
  double FO_mean_time = 0.0;
  double PREP_mean_time = 0.0;
  double AFTER_mean_time = 0.0;
  double LO_max_time = -1e+6;
  double RO_max_time = -1e+6;
  double FW_max_time = -1e+6;
  double FO_max_time = -1e+6;
  double PREP_max_time = -1e+6;
  double AFTER_max_time = -1e+6;

  int LO_iter_tot_count = 0;
  int RO_iter_tot_count = 0;
  int FW_iter_tot_count = 0;
  double LO_average_time_per_iter = 0.0;
  double RO_average_time_per_iter = 0.0;
  double FW_average_time_per_iter = 0.0;
  double LO_max_time_per_iter = 0.0;
  double RO_max_time_per_iter = 0.0;
  double FW_max_time_per_iter = 0.0;

  for (int i = 0; i < 10000; i++ ) {
    ego_ex_(0) = distrib_ego_ey(generator);
    ego_ex_(1) = distrib_ego_ephi(generator);
    ego_ex_(2) = distrib_ego_vx(generator);
    ego_ex_(3) = distrib_ego_vy(generator);
    ego_ex_(4) = 0.0; //distrib_ego_omega(generator);
    ego_ex_(5) = 0.0; //distrib_ego_delta(generator);
    ego_ex_(6) = 0.0;
    ego_ex_(7) = distrib_ego_s(generator);

    opp_ex_(0) = distrib_opp_ey(generator);
    opp_ex_(1) = distrib_opp_ephi(generator);
    opp_ex_(2) = distrib_ego_vx(generator);
    opp_ex_(3) = distrib_ego_vy(generator);
    opp_ex_(4) = 0.0; //distrib_ego_omega(generator);
    opp_ex_(5) = 0.0; //distrib_ego_delta(generator);
    opp_ex_(6) = 0.0;
    opp_ex_(7) = ego_ex_(7) + distrib_opp_d_s(generator);

    Eigen::VectorXd ego_pos(3);
    Eigen::VectorXd debug_ego_state(3);
    debug_ego_state[0] = ego_ex_(7);
    debug_ego_state[1] = ego_ex_(0);
    debug_ego_state[2] = ego_ex_(1);
    transform_to_absolute_pos(ego_pos, debug_ego_state);
    Eigen::VectorXd opp_pos(3);
    Eigen::VectorXd debug_opp_state(3);
    debug_opp_state[0] = opp_ex_(7);
    debug_opp_state[1] = opp_ex_(0);
    debug_opp_state[2] = opp_ex_(1);
    transform_to_absolute_pos(opp_pos, debug_opp_state);
/*
    std::cout << "ego: " << ego_pos(0) << ", " << ego_pos(1) << ", " << ego_pos(2) << std::endl;
    std::cout << "     " << ego_ex_(7) << ", " << ego_ex_(0) << ", " << ego_ex_(1) << std::endl;
    std::cout << "opp: " << opp_pos(0) << ", " << opp_pos(1) << ", " << opp_pos(2) << std::endl;
    std::cout << "     " << opp_ex_(7) << ", " << opp_ex_(0) << ", " << opp_ex_(1) << std::endl;

    std::cout << "ego_ex_(0) = " << ego_ex_(0) << "; \n";
    std::cout << "ego_ex_(1) = " << ego_ex_(1) << "; \n";
    std::cout << "ego_ex_(2) = " << ego_ex_(2) << "; \n";
    std::cout << "ego_ex_(3) = " << ego_ex_(3) << "; \n";
    std::cout << "ego_ex_(4) = " << ego_ex_(4) << "; \n";
    std::cout << "ego_ex_(5) = " << ego_ex_(5) << "; \n";
    std::cout << "ego_ex_(6) = " << ego_ex_(6) << "; \n";
    std::cout << "ego_ex_(7) = " << ego_ex_(7) << "; \n";

    std::cout << "opp_ex_(0) = " << opp_ex_(0) << "; \n";
    std::cout << "opp_ex_(1) = " << opp_ex_(1) << "; \n";
    std::cout << "opp_ex_(2) = " << opp_ex_(2) << "; \n";
    std::cout << "opp_ex_(3) = " << opp_ex_(3) << "; \n";
    std::cout << "opp_ex_(4) = " << opp_ex_(4) << "; \n";
    std::cout << "opp_ex_(5) = " << opp_ex_(5) << "; \n";
    std::cout << "opp_ex_(6) = " << opp_ex_(6) << "; \n";
    std::cout << "opp_ex_(7) = " << opp_ex_(7) << "; \n";
*/
    /*
    Pose2D ego_state {ego_x_(0), ego_x_(1), ego_x_(2)};
    Pose2D opp_state {opp_x_(0), opp_x_(1), opp_x_(2)};

    std::vector<double> current_scan = get_scan(ego_state, opp_state);

    scan = make_lidar_msg(current_scan);
    odom = make_odom_msg(ego_state);

    lidarLogger(scan_ptr);
    poseLogger(odom_ptr);
    oppDetector(t_virtual);
    */
    t_start = std::chrono::high_resolution_clock::now();
    updateCmd(t_virtual);
    t_end = std::chrono::high_resolution_clock::now();
    dt_ms = t_end - t_start;

    file_ << dt_ms.count();
    mean_time = (mean_time * i + dt_ms.count()) / (i + 1);
    if (dt_ms.count()>max_time) max_time = dt_ms.count();

    if (i%1000==0) std::cout << "iter: " << i << ", max_time [ms] " << max_time << std::endl;
/*
    std::cout << "      " << "  " << "  current [ms] " << dt_ms.count() << std::endl;

    std::cout << "      " << "  " << "  LO    [ms] " << LO_time << std::endl;
    std::cout << "      " << "  " << "  RO    [ms] " << RO_time << std::endl;
    std::cout << "      " << "  " << "  FO    [ms] " << FO_time << std::endl;
    std::cout << "      " << "  " << "  PREP  [ms] " << PREP_time << std::endl;
    std::cout << "      " << "  " << "  AFTER [ms] " << AFTER_time << "\n" << std::endl;
    std::cout << "      " << "LO iter: " << LO_iter_num << std::endl;
    std::cout << "      " << "    kkt: " << LO_kkt_val << std::endl;
    std::cout << "      " << "RO iter: " << RO_iter_num << std::endl;
    std::cout << "      " << "    kkt: " << RO_kkt_val << "\n" << std::endl;
*/
    file_ << ", " << LO_time;
    file_ << ", " << LO_iter_num;
    file_ << ", " << LO_kkt_val;
    if (LO_time>LO_max_time) LO_max_time = LO_time;
    LO_mean_time = (LO_mean_time * i + LO_time) / (i + 1);

    file_ << ", " << RO_time;
    file_ << ", " << RO_iter_num;
    file_ << ", " << RO_kkt_val;
    if (RO_time>RO_max_time) RO_max_time = RO_time;
    RO_mean_time = (RO_mean_time * i + RO_time) / (i + 1);

    file_ << ", " << FW_time;
    file_ << ", " << FW_iter_num;
    file_ << ", " << FW_kkt_val;
    if (FW_time>FW_max_time) FW_max_time = FW_time;
    FW_mean_time = (FW_mean_time * i + FW_time) / (i + 1);

    file_ << ", " << FO_time;
    if (FO_time>FO_max_time) FO_max_time = FO_time;
    FO_mean_time = (FO_mean_time * i + FO_time) / (i + 1);

    file_ << ", " << PREP_time;
    if (PREP_time>PREP_max_time) PREP_max_time = PREP_time;
    PREP_mean_time = (PREP_mean_time * i + PREP_time) / (i + 1);

    file_ << ", " << AFTER_time;
    if (AFTER_time>AFTER_max_time) AFTER_max_time = AFTER_time;
    AFTER_mean_time = (AFTER_mean_time * i + AFTER_time) / (i + 1);

    //if (LO_iter_num>0) {
      LO_average_time_per_iter = (LO_average_time_per_iter * LO_iter_tot_count + LO_time) / (LO_iter_tot_count + LO_iter_num);
      LO_iter_tot_count += LO_iter_num;

      if ((LO_time/LO_iter_num)>LO_max_time_per_iter) LO_max_time_per_iter = LO_time/LO_iter_num;
      file_ << ", " << LO_time/LO_iter_num;
    //}
    //else file_ << ", " << -1;

    //if (RO_iter_num>0) {
      RO_average_time_per_iter = (RO_average_time_per_iter * RO_iter_tot_count + RO_time) / (RO_iter_tot_count + RO_iter_num);
      RO_iter_tot_count += RO_iter_num;

      if ((RO_time/RO_iter_num)>RO_max_time_per_iter) RO_max_time_per_iter = RO_time/RO_iter_num;
      file_ << ", " << RO_time/RO_iter_num;
    //}
    //else file_ << ", " << -1 << std::endl;

      FW_average_time_per_iter = (FW_average_time_per_iter * FW_iter_tot_count + FW_time) / (FW_iter_tot_count + FW_iter_num);
      FW_iter_tot_count += FW_iter_num;

      if ((FW_time/FW_iter_num)>FW_max_time_per_iter) FW_max_time_per_iter = FW_time/FW_iter_num;
      file_ << ", " << FW_time/FW_iter_num << std::endl;
  }
  file_ << "---" << std::endl;
  file_ << "mean_time: " << mean_time << std::endl;
  file_ << "max_time: " << max_time << std::endl;

  file_ << "LO_mean_time: " << LO_mean_time << std::endl;
  file_ << "RO_mean_time: " << RO_mean_time << std::endl;
  file_ << "FW_mean_time: " << FW_mean_time << std::endl;
  file_ << "FO_mean_time: " << FO_mean_time << std::endl;
  file_ << "PREP_mean_time: " << PREP_mean_time << std::endl;
  file_ << "AFTER_mean_time: " << AFTER_mean_time << std::endl;


  file_ << "LO_max_time: " << LO_max_time << std::endl;
  file_ << "RO_max_time: " << RO_max_time << std::endl;
  file_ << "FW_max_time: " << FW_max_time << std::endl;
  file_ << "FO_max_time: " << FO_max_time << std::endl;
  file_ << "PREP_max_time: " << PREP_max_time << std::endl;
  file_ << "AFTER_max_time: " << AFTER_max_time << std::endl;

  file_ << "LO_average_time_per_iter: " << LO_average_time_per_iter << std::endl;
  file_ << "RO_average_time_per_iter: " << RO_average_time_per_iter << std::endl;
  file_ << "FW_average_time_per_iter: " << FW_average_time_per_iter << std::endl;
  file_ << "LO_max_time_per_iter: " << LO_max_time_per_iter << std::endl;
  file_ << "RO_max_time_per_iter: " << RO_max_time_per_iter << std::endl;
  file_ << "FW_max_time_per_iter: " << FW_max_time_per_iter << std::endl;

  return 0;
}
