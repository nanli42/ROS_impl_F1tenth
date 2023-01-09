#ifndef F110_OPP_DETECT_NODE_H
#define F110_OPP_DETECT_NODE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include <sstream>
#include <fstream>

#include <thread>
#include <unistd.h>
#include <sys/syscall.h>

// grid map
extern nav_msgs::OccupancyGrid loaded_map;
extern std::vector<std::vector<bool>> grid;
extern double init_time;
extern double last_lidar_receiving_time;

extern sensor_msgs::LaserScan newestLidarMsg;

extern ros::Publisher pub_opp_vis;
extern ros::Publisher pub_filtered_lidar;
extern ros::Publisher pub_opp_odom;
extern ros::Publisher pub_opp_complete;

extern geometry_msgs::PoseArray particles_ros_;

bool requestMap(ros::NodeHandle &nh);
void readMap(const nav_msgs::OccupancyGrid& map);

void oppDetector(const ros::WallTimerEvent& event);
//void oppDetector(const sensor_msgs::LaserScan::ConstPtr& msg);
//void oppDetector(const std_msgs::Bool::ConstPtr& msg);
void lidarLogger(const sensor_msgs::LaserScan::ConstPtr& msg);
void poseLogger(const nav_msgs::Odometry::ConstPtr& msg);

void set_main_thread_sched_priority(int core_id, int sched_priority, int policy);
void set_sub_thread_sched_priority(int core_id, int sched_priority, int policy, std::thread& th);
void set_only_priority(int sched_priority, int policy, std::thread& th);

geometry_msgs::Quaternion toQuaternion(double yaw);
double toEulerAngle(geometry_msgs::Quaternion q);
std::vector<int> world_to_map(double x, double y);
void visualize_opp(std::vector<std::vector<double>> opp_points);
void publish_filtered_lidar(sensor_msgs::LaserScan filtered_lidar);

//void init_pf(nav_msgs::OccupancyGrid loaded_map);
//void exec_pf(sensor_msgs::LaserScan msg);
#endif
