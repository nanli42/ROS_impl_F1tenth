#ifndef PARTICLE_FILTER_TYPE_H
#define PARTICLE_FILTER_TYPE_H

#include <thread>

#include <random>
#include <vector>
#include <algorithm>
#include <iostream>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


#include <iostream>
#include <fstream>

typedef struct {
	float x;
	float y;
	float theta;
} robot_state;

typedef struct {
	float x;
	float y;
	float theta;
	float weight;
} particle_state;

#include "range_libc/RangeLib.h"

#define PARTICLE_NUM 1000
#define INIT_POSE_X 0.0
#define INIT_POSE_Y 0.0
#define INIT_POSE_YAW 0.0
#define MOTION_DISPERSION_X 0.1
#define MOTION_DISPERSION_Y 0.1
#define MOTION_DISPERSION_THETA 0.25

using namespace ranges;

class ParticleFilter {
public:
	ParticleFilter(nav_msgs::OccupancyGrid loaded_map);
	~ParticleFilter();
	void precompute_sensor_model();

	//void MCL(const std_msgs::Bool::ConstPtr& msg);
	void MCL();
	void resample();
	void motion_model(double vx, double vy, double w);
	void sensor_model();

	void pub_odom();

	void particles_vis();
	void expected_pose_vis();

	ros::Publisher publish_particlecloud_;
	ros::Publisher publish_expected_pose_;
	ros::Publisher publish_odom_;
	ros::Publisher publish_pf_complete_;

	sensor_msgs::LaserScan newestLidarMsg;
	sensor_msgs::LaserScan currentLidarMsg;
	ackermann_msgs::AckermannDriveStamped newestDriveMsg;
	ackermann_msgs::AckermannDriveStamped currentDriveMsg;

private:
	std::vector<particle_state> particles_;
	std::vector<double> init_pose_;
	int numParticles_;
	double motion_disp_x;
	double motion_disp_y;
	double motion_disp_theta;
	int table_width;
	float max_range_px;
	particle_state expected_pose;

};

void set_main_thread_sched_priority(int core_id, int sched_priority, int policy);
void set_sub_thread_sched_priority(int core_id, int sched_priority, int policy, std::thread& th);
void set_only_priority(int sched_priority, int policy, std::thread& th);

#endif
