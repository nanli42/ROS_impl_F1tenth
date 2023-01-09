#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>

#include <sstream>
#include <fstream>

#include <thread>

using namespace std::chrono;
ros::Publisher pub_opp_detect;
ros::Publisher pub_particle_filter;
ros::Publisher pub_nmpc;
ros::Publisher pub_release;

std::ofstream file_trace_;

void set_main_thread_sched_priority(int core_id, int sched_priority, int policy);
void set_sub_thread_sched_priority(int core_id, int sched_priority, int policy, std::thread& th);

double get_time() {
  auto t_instant = high_resolution_clock::now();
  auto t_instant_millis = std::chrono::duration_cast<std::chrono::microseconds>(t_instant.time_since_epoch()).count();
  double t_instant_ms = (t_instant_millis % 1000000) / 1000.0;
  return t_instant_ms;
}
void sched(const ros::WallTimerEvent& event) {
  struct timespec ts;
  struct timeval utime;
  struct timeval stime;
  ts.tv_sec = 0;
  ts.tv_nsec = 0;

  //ROS_INFO("--- start of timer manager ---");
  std_msgs::Bool msg;
  msg.data = true;

  auto t11 = std::chrono::high_resolution_clock::now();
  pub_release.publish(msg);
  //ROS_INFO("release_ctrl: %7.3f [ms]", get_time());
  pub_opp_detect.publish(msg);
  auto t12 = std::chrono::high_resolution_clock::now();

  //ROS_INFO("OD: %7.3f [ms]", get_time());
  //usleep(1000);
  ts.tv_nsec = 1000000;
  nanosleep(&ts, NULL);

  auto t21 = std::chrono::high_resolution_clock::now();
  pub_particle_filter.publish(msg);
  auto t22 = std::chrono::high_resolution_clock::now();

  //ROS_INFO("PF: %7.3f [ms]", get_time());
  //usleep(7000);
  ts.tv_nsec = 7000000;
  nanosleep(&ts, NULL);

  auto t31 = std::chrono::high_resolution_clock::now();
  pub_nmpc.publish(msg);
  auto t32 = std::chrono::high_resolution_clock::now();

  //ROS_INFO("NMPC: %7.3f [ms]", get_time());

  file_trace_ << t11.time_since_epoch().count() << ", " << t12.time_since_epoch().count() << std::endl;
  file_trace_ << t21.time_since_epoch().count() << ", " << t22.time_since_epoch().count() << std::endl;
  file_trace_ << t31.time_since_epoch().count() << ", " << t32.time_since_epoch().count() << std::endl;
}

int main(int argc, char **argv) {
  //file_trace_.open("/home/nvidia/test_by_Nan_Li/timing/TM_trace.txt");
  file_trace_.open("/home/nan/workspace/project/run.f1tenth/timing/TM_trace.txt");

  // init ros
  ros::init(argc, argv, "timer_manager");

  ros::NodeHandle nh;
  ros::CallbackQueue queue_timer;
  nh.setCallbackQueue(&queue_timer);

  pub_opp_detect = nh.advertise<std_msgs::Bool>("sched/timer_opp_detect", 1);
  pub_particle_filter = nh.advertise<std_msgs::Bool>("sched/timer_particle_filter", 1);
  pub_nmpc = nh.advertise<std_msgs::Bool>("sched/timer_nmpc", 1);
  pub_release = nh.advertise<std_msgs::Bool>("sched/timer_release_ctrl", 1);

  double qp_calc_period;
  nh.getParam("/qp_calc_period", qp_calc_period);
  ros::WallTimer timer = nh.createWallTimer(ros::WallDuration(qp_calc_period), sched);

  std::thread thread_timer([&queue_timer]() {
      ros::SingleThreadedSpinner spinner_timer;
      spinner_timer.spin(&queue_timer);
  });
  set_sub_thread_sched_priority(0, 99, SCHED_RR, thread_timer);
  thread_timer.join();

  return 0;
}
