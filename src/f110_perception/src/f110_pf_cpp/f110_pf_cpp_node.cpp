#include "f110_pf_cpp_node.hpp"

ParticleFilter* pf;

std::ofstream file_;
std::ofstream file_trace_;
std::chrono::time_point<std::chrono::high_resolution_clock> chrono_init_time;

nav_msgs::OccupancyGrid loaded_map;
bool requestMap(ros::NodeHandle &nh);
void lidarLogger(const sensor_msgs::LaserScan::ConstPtr& msg);
void driveLogger(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
void filteredLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

int main(int argc, char **argv) {
//============================================================================//
  file_.open("/home/nan/workspace/project/run.f1tenth/timing/pf_cpp.txt");
  file_trace_.open("/home/nan/workspace/project/run.f1tenth/timing/PF_trace.txt");
  //file_trace_.open("/home/nvidia/test_by_Nan_Li/timing/PF_trace.txt");
//============================================================================//

  ros::init(argc, argv, "pf_cpp");

  ros::NodeHandle nh;
  if (!requestMap(nh))
    exit(-1);

//============================================================================//
  auto start_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(start_time-chrono_init_time);
  file_ << time_span.count() << " - " << ros::Time::now() << std::endl;
//============================================================================//

  ros::Time::init();
  pf = new ParticleFilter(loaded_map);
  pf->precompute_sensor_model();

  //ros::NodeHandle nh_lidar_logger;
  //ros::NodeHandle nh_pf_cpp;
  ros::NodeHandle nh_drive_logger;
  ros::NodeHandle nh_filtered_lidar;

  ros::NodeHandle nh_particles_vis;
  ros::NodeHandle nh_exp_pose_vis;
  ros::NodeHandle nh_pub_odom;
  ros::NodeHandle nh_pub_pf_complete;

  //ros::CallbackQueue queue_lidar_logger;
  //nh_lidar_logger.setCallbackQueue(&queue_lidar_logger);
  //ros::CallbackQueue queue_pf_cpp;
  //nh_pf_cpp.setCallbackQueue(&queue_pf_cpp);
  ros::CallbackQueue queue_drive_logger;
  nh_drive_logger.setCallbackQueue(&queue_drive_logger);
  ros::CallbackQueue queue_filtered_lidar;
  nh_filtered_lidar.setCallbackQueue(&queue_filtered_lidar);

  //ros::Subscriber sub_lidar_log = nh_lidar_logger.subscribe("ego_id/scan", 1, lidarLogger, ros::TransportHints().tcpNoDelay(true));
  //ros::Subscriber sub_lidar_log = nh_lidar_logger.subscribe("OD/filtered_lidar", 1, lidarLogger, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber sub_drive_log = nh_drive_logger.subscribe("ego_id/drive", 1, driveLogger, ros::TransportHints().tcpNoDelay(true));

  /*
  double qp_calc_period;
  nh.getParam("/qp_calc_period", qp_calc_period);
  ros::WallTimer timer_pf_cpp = nh_pf_cpp.createWallTimer(ros::WallDuration(qp_calc_period), std::bind(&ParticleFilter::MCL, pf));
  chrono_init_time = std::chrono::high_resolution_clock::now();
  */
  //ros::Subscriber timer_pf_cpp = nh_pf_cpp.subscribe("sched/timer_particle_filter", 1, &ParticleFilter::MCL, pf, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber sub_filtered_lidar = nh_filtered_lidar.subscribe("filtered_lidar", 1, filteredLidarCallback, ros::TransportHints().tcpNoDelay(true));

  pf->publish_particlecloud_ = nh_particles_vis.advertise<geometry_msgs::PoseArray>("PF/particle_cloud", 1, true);
  pf->publish_expected_pose_ = nh_exp_pose_vis.advertise<geometry_msgs::PoseStamped>("PF/expected_pose", 1, true);
  pf->publish_odom_ = nh_pub_odom.advertise<nav_msgs::Odometry>("pf_est_odom", 1, true);
  //pf->publish_pf_complete_ = nh_pub_pf_complete.advertise<std_msgs::Bool>("sched/timer_nmpc", 1);

  /*
  std::thread thread_timer_pf_cpp([&queue_pf_cpp]() {
      ros::SingleThreadedSpinner spinner_timer_pf_cpp;
      spinner_timer_pf_cpp.spin(&queue_pf_cpp);
  });
  set_sub_thread_sched_priority(0, 99, SCHED_RR, thread_timer_pf_cpp);
  */
  /*
  std::thread thread_sub_lidar_log([&queue_lidar_logger]() {
        ros::SingleThreadedSpinner spinner_sub_lidar_log;
        spinner_sub_lidar_log.spin(&queue_lidar_logger);
  });
  //set_sub_thread_sched_priority(3, 90, SCHED_RR, thread_sub_lidar_log);
  */
  std::thread thread_sub_drive_log([&queue_drive_logger]() {
        ros::SingleThreadedSpinner spinner_sub_drive_log;
        spinner_sub_drive_log.spin(&queue_drive_logger);
  });
  set_only_priority(99, SCHED_RR, thread_sub_drive_log);

  std::thread thread_sub_filtered_lidar([&queue_filtered_lidar]() {
        ros::SingleThreadedSpinner spinner_sub_filtered_lidar;
        spinner_sub_filtered_lidar.spin(&queue_filtered_lidar);
  });
  set_only_priority(99, SCHED_RR, thread_sub_filtered_lidar);

  //thread_timer_pf_cpp.join();
  //thread_sub_lidar_log.join();
  thread_sub_drive_log.join();
  thread_sub_filtered_lidar.join();

  return 0;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

bool requestMap(ros::NodeHandle &nh) {
 nav_msgs::GetMap::Request req;
 nav_msgs::GetMap::Response res;

 while (!ros::service::waitForService("static_map", ros::Duration(3.0)));

 ROS_INFO("Requesting the map...");
 ros::ServiceClient mapClient = nh.serviceClient<nav_msgs::GetMap>("static_map");

 if (mapClient.call(req, res)) {
   loaded_map = res.map;
   ROS_INFO("Received a %d X %d map @ %.3f m/px\n",
     loaded_map.info.height,
     loaded_map.info.width,
     loaded_map.info.resolution
   );
   return true;
 } else {
   ROS_ERROR("Fail to call map service!");
   return false;
 }
}
/*
void lidarLogger(const sensor_msgs::LaserScan::ConstPtr& msg) {
  pf->newestLidarMsg = *msg;
}
*/
void driveLogger(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
  pf->newestDriveMsg = *msg;
}

void filteredLidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
//============================================================================//
  auto start_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(start_time-chrono_init_time);
  file_ << time_span.count() << ", ";
//============================================================================//

  pf->newestLidarMsg = *msg;

  pf->MCL();

  auto t_start_millis = std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count();
  double t_start_ms = (t_start_millis % 10000000) / 10000.0;
  ROS_INFO("0--- start of pf ...: %7.3f [ms]\n", t_start_ms);

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_end_millis = std::chrono::duration_cast<std::chrono::microseconds>(t_end.time_since_epoch()).count();
  double t_end_ms = (t_end_millis % 10000000) / 10000.0;
  ROS_INFO("1--- end of pf ...: %7.3f [ms]\n", t_end_ms);
}
