// multi-thread ref:
// https://nicolovaligi.com/articles/concurrency-and-parallelism-in-ros1-and-ros2-application-apis/
// style guide ref:
// http://wiki.ros.org/CppStyleGuide

#include "nmpc_ctrl.hpp"

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
Eigen::VectorXd ego_ex_snapshot_LO = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)
Eigen::VectorXd opp_ex_snapshot_LO = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)
Eigen::VectorXd ego_ex_snapshot_RO = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)
Eigen::VectorXd opp_ex_snapshot_RO = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)
Eigen::VectorXd ego_ex_snapshot_FO = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)
Eigen::VectorXd opp_ex_snapshot_FO = Eigen::VectorXd::Zero(8); //position of the vehicle (ey,ephi,vx,vy,omega,s)

ros::Subscriber sub_ego_odom;
ros::Subscriber sub_opp_odom;

ros::Publisher cmd_pub;
ros::Publisher traj_marker_pub;
ros::Publisher pos_marker_pub;
//ros::Publisher odom_delta_pub;
ros::Publisher compensated_pose_pub;
ros::Publisher pub_ctrler_complete;
ros::Publisher pub_trigger_RO;

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
bool RESP = true;

std::ofstream file_;
std::ofstream file_trace_;
std::ofstream file_trace_LO_FW_;
std::ofstream file_trace_RO_;
std::ofstream file_trace_FO_;

std::chrono::time_point<std::chrono::high_resolution_clock> chrono_init_time;

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

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "nmpc_ctrl_node");

  // def node handler
  ros::NodeHandle nh_ctrl;
  ros::NodeHandle nh_ctrl_LO_FW;
  ros::NodeHandle nh_ctrl_RO;
  ros::NodeHandle nh_ctrl_FO;
  ros::NodeHandle nh_ctrl_release_ctrl;
  ros::NodeHandle nh_cmd_pub;
  ros::NodeHandle nh_odom_ego;
  ros::NodeHandle nh_odom_opp;
  ros::NodeHandle nh_odom_delta;
  ros::NodeHandle nh_ctrler_complete;
  ros::NodeHandle nh_trigger_RO;

  // set callback queue
  ros::CallbackQueue queue_timer_ctrl;
  ros::CallbackQueue queue_timer_LO_FW;
  ros::CallbackQueue queue_timer_RO;
  ros::CallbackQueue queue_timer_FO;
  ros::CallbackQueue queue_timer_release_ctrl;
  ros::CallbackQueue queue_sub_ego_odom;
  ros::CallbackQueue queue_sub_opp_odom;

  nh_ctrl.setCallbackQueue(&queue_timer_ctrl);
  nh_ctrl_LO_FW.setCallbackQueue(&queue_timer_LO_FW);
  nh_ctrl_RO.setCallbackQueue(&queue_timer_RO);
  nh_ctrl_FO.setCallbackQueue(&queue_timer_FO);
  nh_ctrl_release_ctrl.setCallbackQueue(&queue_timer_release_ctrl);
  nh_odom_ego.setCallbackQueue(&queue_sub_ego_odom);
  nh_odom_opp.setCallbackQueue(&queue_sub_opp_odom);

  // load param for timer
  nh_ctrl.getParam("/qp_calc_period", qp_calc_period);
  nh_ctrl_LO_FW.getParam("/qp_calc_period", qp_calc_period);
  nh_ctrl_RO.getParam("/qp_calc_period", qp_calc_period);
  ROS_DEBUG("qp_calc_period = %f", qp_calc_period);

  // load identification
  nh_ctrl.getParam("/who_am_i", who_am_i);

  if (who_am_i==0) { //ego
  //============================================================================//
    chrono_init_time = std::chrono::high_resolution_clock::now();
    //file_.open("/home/nvidia/test_by_Nan_Li/timing/nmpc_ctrl.txt");
    //file_.open("/home/nan/workspace/project/run.f1tenth/timing/nmpc_ctrl.txt");

    std::string str1 = "/home/nan/workspace/project/run.f1tenth";
    //std::string str1 = "/home/nvidia/test_by_Nan_Li";
    file_trace_.open(str1 + "/timing/NMPC_trace.txt");
    file_trace_LO_FW_.open(str1 + "/timing/NMPC_LO_FW_trace.txt");
    file_trace_RO_.open(str1 + "/timing/NMPC_RO_trace.txt");
    file_trace_FO_.open(str1 + "/timing/NMPC_FO_trace.txt");
  //============================================================================//
  //============================================================================//
    /*
    auto start_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(start_time-chrono_init_time);
    file_ << time_span.count() << " - " << ros::Time::now() << std::endl;
    */
  //============================================================================//
  }

  if (who_am_i==0) {
    //set_main_thread_sched_priority(5, 98, SCHED_RR);
  }

  // init QP solver
  acado_initializeSolver();
  acadoVariables_opp = acadoVariables;

  // create the timer
  //ros::Timer timer_ctrl = nh_ctrl.createTimer(ros::Duration(qp_calc_period), updateCmd);
  /*
  ros::Timer timer_LO_FW = nh_ctrl_LO_FW.createTimer(ros::Duration(qp_calc_period), calcLOFW);
  ros::Timer timer_RO = nh_ctrl_RO.createTimer(ros::Duration(qp_calc_period), calcRO);
  */
  ros::Subscriber timer_LO_FW;
  ros::Subscriber timer_RO;
  ros::Subscriber timer_FO;
  ros::Subscriber timer_release_ctrl;
  if (who_am_i==0) {

    //timer_LO_FW = nh_ctrl_LO_FW.subscribe("sched/timer_nmpc", 1, calcLOFW, ros::TransportHints().tcpNoDelay(true)); // subscriber to position of the vehicle
    timer_LO_FW = nh_ctrl_LO_FW.subscribe("/odom_ego", 1, calcLOFW, ros::TransportHints().tcpNoDelay(true)); // subscriber to position of the vehicle
    //timer_RO = nh_ctrl_RO.subscribe("trigger_RO", 1, calcRO, ros::TransportHints().tcpNoDelay(true)); // subscriber to position of the vehicle
    //timer_FO = nh_ctrl_FO.subscribe("sched/timer_nmpc", 1, calcFO, ros::TransportHints().tcpNoDelay(true)); // subscriber to position of the vehicle
    //timer_release_ctrl = nh_ctrl_release_ctrl.subscribe("sched/timer_release_ctrl", 1, release_ctrl, ros::TransportHints().tcpNoDelay(true)); // subscriber to position of the vehicle
  }
  if (who_am_i==1) {
    sub_ego_odom = nh_odom_ego.subscribe("/odom_ego", 1, egoOdomCallback, ros::TransportHints().tcpNoDelay(true)); // subscriber to position of the vehicle
  }

  // set subscriber
  /*
  ros::SubscribeOptions ops_ego;
  ops_ego.template init<nav_msgs::Odometry>("/odom_ego", 1, egoOdomCallback); // subscriber to position of the vehicle
  ops_ego.allow_concurrent_callbacks = true;
  sub_ego_odom = nh_odom_ego.subscribe(ops_ego);
  ros::SubscribeOptions ops_opp;
  ops_opp.template init<nav_msgs::Odometry>("/odom_opp", 1, oppOdomCallback); // subscriber to position of the vehicle
  ops_opp.allow_concurrent_callbacks = false;
  sub_opp_odom = nh_odom_opp.subscribe(ops_opp);
  */
  // set subscriber
  //sub_ego_odom = nh_odom_ego.subscribe("/odom_ego", 1, egoOdomCallback, ros::TransportHints().tcpNoDelay(true)); // subscriber to position of the vehicle
  sub_opp_odom = nh_odom_opp.subscribe("/odom_opp", 1, oppOdomCallback, ros::TransportHints().tcpNoDelay(true)); // subscriber to position of the vehicle

  init_event.current_real = ros::Time::now();

  // set publisher
  cmd_pub = nh_cmd_pub.advertise<ackermann_msgs::AckermannDriveStamped>("NMPC/drive_nmpc", 10);
  traj_marker_pub = nh_cmd_pub.advertise<visualization_msgs::Marker>("/traj_marker", 10); //publisher marker on rviz
  pos_marker_pub = nh_cmd_pub.advertise<visualization_msgs::Marker>("/pos_marker", 10); //publisher marker on rviz
  compensated_pose_pub = nh_odom_delta.advertise<geometry_msgs::PoseStamped>("NMPC/compensated_pose", 1);

  if (who_am_i==0) {
    std::thread thread_timer_LO_FW([&queue_timer_LO_FW]() {
        ros::SingleThreadedSpinner spinner_timer_LO_FW;
        spinner_timer_LO_FW.spin(&queue_timer_LO_FW);
    });

    std::thread thread_sub_opp_odom([&queue_sub_opp_odom]() {
        ros::SingleThreadedSpinner spinner_sub_opp_odom;
        spinner_sub_opp_odom.spin(&queue_sub_opp_odom);
    });

    set_only_priority(99, SCHED_RR, thread_timer_LO_FW);
    set_only_priority(99, SCHED_RR, thread_sub_opp_odom);

    thread_timer_LO_FW.join();
    thread_sub_opp_odom.join();
  }

  if (who_am_i==1) {
    std::thread thread_sub_ego_odom([&queue_sub_ego_odom]() {
        ros::SingleThreadedSpinner spinner_sub_ego_odom;
        spinner_sub_ego_odom.spin(&queue_sub_ego_odom);
    });

    std::thread thread_sub_opp_odom([&queue_sub_opp_odom]() {
        ros::SingleThreadedSpinner spinner_sub_opp_odom;
        spinner_sub_opp_odom.spin(&queue_sub_opp_odom);
    });

    thread_sub_ego_odom.join();
    thread_sub_opp_odom.join();
  }

  return 0;
}
