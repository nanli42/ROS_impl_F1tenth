#include "f110_opp_detect_node.hpp"

std::vector<std::vector<bool>> grid;
double init_time;
double last_lidar_receiving_time = 0;
sensor_msgs::LaserScan newestLidarMsg;
nav_msgs::OccupancyGrid loaded_map;
ros::Publisher pub_opp_vis;
ros::Publisher pub_filtered_lidar;
ros::Publisher pub_opp_odom;
ros::Publisher pub_opp_complete;

std::ofstream file_;
std::ofstream file_trace_;
std::chrono::time_point<std::chrono::high_resolution_clock> chrono_init_time;

sensor_msgs::LaserScan filteredLidarMsg;

int main(int argc, char **argv) {
//============================================================================//
  chrono_init_time = std::chrono::high_resolution_clock::now();

  //sysctl -w kernel.sched_rt_runtime_us=-1
  //file_.open("/home/nvidia/test_by_Nan_Li/timing/opp_detect.txt");
  file_.open("/home/nan/workspace/project/run.f1tenth/timing/opp_detect.txt");

  file_trace_.open("/home/nan/workspace/project/run.f1tenth/timing/OD_trace.txt");
  //file_trace_.open("/home/nvidia/test_by_Nan_Li/timing/OD_trace.txt");
//============================================================================//

  ros::init(argc, argv, "opp_detect");

  // load a map
  // ref: https://slideplayer.com/slide/12407790/, page 47
  // Lecturer: Roi Yehoshua, 2016
  ros::NodeHandle nh;
  if (!requestMap(nh))
    exit(-1);

  // init ros
  ros::Time::init();
  init_time = ros::Time::now().toSec();;

  //init_pf(loaded_map);

  // TODO:
  // timer -> detect whether exist opp
  // if yes, find the center of opp & send corrected LiDAR msg to PF
  // if no, send original LiDAR msg to PF
  // add a tracing system

  ros::NodeHandle nh_lidar_logger;
  ros::NodeHandle nh_pose_logger;
  ros::NodeHandle nh_opp_detect;
  //ros::NodeHandle nh_wcet_lidar;

  ros::NodeHandle nh_opp_vis;
  ros::NodeHandle nh_filtered_lidar;
  ros::NodeHandle nh_opp_odom;
  //ros::NodeHandle nh_opp_complete;

  ros::CallbackQueue queue_lidar_logger;
  nh_lidar_logger.setCallbackQueue(&queue_lidar_logger);
  ros::CallbackQueue queue_pose_logger;
  nh_pose_logger.setCallbackQueue(&queue_pose_logger);
  ros::CallbackQueue queue_timer;
  nh_opp_detect.setCallbackQueue(&queue_timer);
  /*
  ros::CallbackQueue queue_wcet_lidar;
  nh_wcet_lidar.setCallbackQueue(&queue_wcet_lidar);
  */

  /*
  tcpNoDelay option is neccesarry, as suggested in
  https://answers.ros.org/question/220129/message-size-of-520-bytes-result-in-inconsistent-publish-timing/

  What I observed:
  - if f1tenth_gym send LiDAR signal less than 50 Hz, what we get will be at 0.02, 0.04, 0.06 ...
  - if more than 50 Hz, we get sth like 0.1, 0.01, 0.01, ..., 0.1, 0.01, 0.01, ...

  In the above link, I noticed that the msg size metters and send a empty scan msg in cmd line
  it shows that this time we can get normal behavoir at >50 Hz, but still there is a limit freq it goes wired...

  By reading the ROS source code, I find that this opt is related to socket:
  ros1-indigo-official/clients/roscpp/src/libros/transport/transport_tcp.cpp, line 164:
    int result = setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
  It seems that the problem is caused by Nagle algo (whether data sending is acknowledged etc) in TCP mechanism:
  "...disabling Nagle algorithm would improve response time..."
  ref: https://www.ibm.com/docs/en/zos/2.3.0?topic=calls-setsockopt
  */
  ros::Subscriber sub_lidar_log = nh_lidar_logger.subscribe("ego_id/scan", 1, lidarLogger, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber sub_pose_log = nh_pose_logger.subscribe("pf/pose/odom", 1, poseLogger, ros::TransportHints().tcpNoDelay(true));


  double qp_calc_period;
  nh.getParam("/qp_calc_period", qp_calc_period);
  ros::WallTimer timer_opp_detect = nh_opp_detect.createWallTimer(ros::WallDuration(qp_calc_period/2), oppDetector);
  //ros::Subscriber sub_opp_detect = nh_opp_detect.subscribe("sched/timer_opp_detect", 1, oppDetector, ros::TransportHints().tcpNoDelay(true));
  //ros::Subscriber sub_wcet_lidar = nh_wcet_lidar.subscribe("ego_id/scan", 1, oppDetector, ros::TransportHints().tcpNoDelay(true));

  pub_opp_vis = nh_opp_vis.advertise<visualization_msgs::Marker>("OD/opp_pos_marker", 1);
  pub_filtered_lidar = nh_filtered_lidar.advertise<sensor_msgs::LaserScan>("OD/filtered_lidar", 1);
  pub_opp_odom = nh_opp_odom.advertise<nav_msgs::Odometry>("OD/od_opp_odom", 1);
  //pub_opp_complete = nh_opp_complete.advertise<std_msgs::Bool>("sched/timer_particle_filter", 1);

  std::thread thread_sub_lidar_log([&queue_lidar_logger]() {
        ros::SingleThreadedSpinner spinner_sub_lidar_log;
        spinner_sub_lidar_log.spin(&queue_lidar_logger);
  });
  set_only_priority(99, SCHED_RR, thread_sub_lidar_log);

  std::thread thread_sub_pose_log([&queue_pose_logger]() {
        ros::SingleThreadedSpinner spinner_sub_pose_log;
        spinner_sub_pose_log.spin(&queue_pose_logger);
  });
  //set_sub_thread_sched_priority(4, 98, SCHED_RR, thread_sub_pose_log);
  set_only_priority(99, SCHED_RR, thread_sub_pose_log);

  std::thread thread_timer_opp_detect([&queue_timer]() {
      ros::SingleThreadedSpinner spinner_timer_opp_detect;
      spinner_timer_opp_detect.spin(&queue_timer);
  });
  //set_sub_thread_sched_priority(0, 99, SCHED_RR, thread_timer_opp_detect);
  set_only_priority(99, SCHED_RR, thread_timer_opp_detect);
/*
  std::thread thread_wcet_lidar([&queue_wcet_lidar]() {
    ros::SingleThreadedSpinner spinner_wcet_lidar;
    spinner_wcet_lidar.spin(&queue_wcet_lidar);
  });
  set_sub_thread_sched_priority(4, 99, SCHED_RR, thread_wcet_lidar);
*/
  thread_sub_lidar_log.join();
  thread_sub_pose_log.join();
  thread_timer_opp_detect.join();
  //thread_wcet_lidar.join();

  return 0;
}
