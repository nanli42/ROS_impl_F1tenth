#ifndef NMPC_CTRL_HPP
#define NMPC_CTRL_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include <thread>
#include <mutex>
#include <iostream>
#include <fstream>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "acado_LO_common.h"
#include "acado_LO_auxiliary_functions.h"
#include "acado_RO_common.h"
#include "acado_RO_auxiliary_functions.h"

using namespace std::chrono;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */
#define STEP_SIZE   0.75

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

typedef std::vector<double> state_type;

extern double qp_calc_period;
extern int who_am_i;
extern ros::TimerEvent init_event;

extern Eigen::VectorXd ego_x_; //position of the vehicle (x,y,phi,v,omega)
extern Eigen::VectorXd ego_ex_; //position of the vehicle (ey,ephi,vx,vy,omega,s)

extern Eigen::VectorXd opp_x_; //position of the vehicle (x,y,phi,v,omega)
extern Eigen::VectorXd opp_ex_; //position of the vehicle (ey,ephi,vx,vy,omega,s)

extern Eigen::VectorXd ego_ex_snapshot_; //position of the vehicle (ey,ephi,vx,vy,omega,s)
extern Eigen::VectorXd opp_ex_snapshot_; //position of the vehicle (ey,ephi,vx,vy,omega,s)
extern Eigen::VectorXd ego_ex_snapshot_LO; //position of the vehicle (ey,ephi,vx,vy,omega,s)
extern Eigen::VectorXd opp_ex_snapshot_LO; //position of the vehicle (ey,ephi,vx,vy,omega,s)
extern Eigen::VectorXd ego_ex_snapshot_RO; //position of the vehicle (ey,ephi,vx,vy,omega,s)
extern Eigen::VectorXd opp_ex_snapshot_RO; //position of the vehicle (ey,ephi,vx,vy,omega,s)
extern Eigen::VectorXd ego_ex_snapshot_FO; //position of the vehicle (ey,ephi,vx,vy,omega,s)
extern Eigen::VectorXd opp_ex_snapshot_FO; //position of the vehicle (ey,ephi,vx,vy,omega,s)

extern ros::Subscriber sub_odom ;
extern ros::Publisher cmd_pub;
extern ros::Publisher traj_marker_pub;
extern ros::Publisher pos_marker_pub;
extern ros::Publisher compensated_pose_pub;
extern ros::Publisher pub_trigger_RO;

extern ros::Publisher pub_ctrler_complete;

extern ACADOvariables acadoVariables;
extern ACADOworkspace acadoWorkspace;

extern ACADOvariables acadoVariables_opp;
extern ACADOvariables acadoVariables_LO;
extern ACADOworkspace acadoWorkspace_LO;
extern ACADOvariables acadoVariables_RO;
extern ACADOworkspace acadoWorkspace_RO;

extern double lr;
extern double lf;

extern double KKT_update_cmd;

extern Eigen::VectorXd prev_cmd; // speed, steering_angle, a, vd
extern Eigen::VectorXd curr_cmd; // speed, steering_angle, a, vd

extern ros::Time ego_odom_time_stamp;

extern std::mutex qp_mutex;

#define FOLLOWING_mode 0
#define LEADING_mode 1
extern int state_machine_current_mode;
extern int state_machine_previous_mode;
extern int time_count_F2L;
extern int time_count_L2F;
extern bool RESP;

extern double LO_iter_counter;
extern double LO_time_counter;
extern double LO_time_max;
extern double LO_time_min;
extern double RO_iter_counter;
extern double RO_time_counter;
extern double RO_time_max;
extern double RO_time_min;
extern double FO_iter_counter;
extern double FO_time_counter;
extern double FO_time_max;
extern double FO_time_min;

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void egoOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void oppOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void updateCmd(const ros::TimerEvent& event);

//void calcLOFW(const std_msgs::Bool::ConstPtr& msg);
void calcLOFW(const nav_msgs::Odometry::ConstPtr& msg);

//void calcRO(const std_msgs::Bool::ConstPtr& msg);
//void calcFO(const std_msgs::Bool::ConstPtr& msg);
//void release_ctrl(const std_msgs::Bool::ConstPtr& msg);
void calcRO();
void calcFO();
void release_ctrl();

void func_LO(bool& result);
void func_RO(bool& result);
void func_FW(bool& result);
void func_FO(bool& result);
bool qp_solver(Eigen::VectorXd init_state);
bool qp_solver_LO(Eigen::VectorXd init_state);
bool qp_solver_RO(Eigen::VectorXd init_state);
void set_constraint_LO();
void set_constraint_RO();

void transform_ego_odom_to_relative_pos(Eigen::VectorXd & x_, Eigen::VectorXd & ex_);
void transform_opp_odom_to_relative_pos(Eigen::VectorXd & x_, Eigen::VectorXd & ex_);
void transform_to_absolute_pos(Eigen::VectorXd & pos, Eigen::VectorXd & state);
void show_trajectory_in_rviz(Eigen::MatrixXd & points, double point_size, int red);
void show_position_in_rviz(Eigen::MatrixXd & points, double point_size, int red);
void visu_traj();

void obs_odes( const state_type &x , state_type &dxdt , double t);
void integrate_in_term_of_t(state_type &x, double delta_t);
double get_curature(double s);
void delay_compensation(Eigen::VectorXd & ex_, double dt);
void check_mode();

void set_main_thread_sched_priority(int core_id, int sched_priority, int policy);
void set_sub_thread_sched_priority(int core_id, int sched_priority, int policy, std::thread& th);
void set_only_priority(int sched_priority, int policy, std::thread& th);
#endif
