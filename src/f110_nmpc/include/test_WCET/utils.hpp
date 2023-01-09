#include "scan_simulator_2d.hpp"
#include "f110_opp_detect_node.hpp"
#include "nmpc_ctrl.hpp"

#include <eigen3/Eigen/Dense>
#include <random>

using namespace std;
using namespace racecar_simulator;

#define PI 3.141592653

double get_range(const Pose2D &pose, double beam_theta, Eigen::Vector2d la, Eigen::Vector2d lb);
void ray_cast_opponents(std::vector<double> &scan, std::vector<double> &scan_angles, const Pose2D &scan_pose, Pose2D &op_pose);

void init_sim();
std::vector<double> get_scan(Pose2D ego_state, Pose2D opp_state);
sensor_msgs::LaserScan make_lidar_msg(std::vector<double> current_scan);
nav_msgs::Odometry make_odom_msg(Pose2D ego_state);
void transform_from_relative_to_absolute_pos(Eigen::VectorXd & pos, Eigen::VectorXd & state);
