#include "nmpc_ctrl.hpp"

#define TEST_WCET 0

#ifdef TEST_WCET
  extern double FO_time;
  extern double PREP_time;
  extern double AFTER_time;
#endif

extern std::ofstream file_;
extern std::ofstream file_trace_;
extern std::chrono::time_point<std::chrono::high_resolution_clock> chrono_init_time;

bool calibration_nmpc = false;

void egoOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  //============================================================================//
    auto start_time = std::chrono::high_resolution_clock::now();
  //============================================================================//

  auto millis = std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count();
  double t_ms = (millis % 1000000) / 1000.0;
  ROS_INFO("--- in nmpc - egoOdomCallback ...: %7.3f [ms]", t_ms);

  /* ATTENTION:
  In simulator:
    msg->twist.twist.linear.x --> velocity at CoG (.y=0)
      see old version f1tenth_gym: src/racecar.cpp#L462
      see new version f1tenth_gym: gym/f110_gym/envs/base_classes.py#L499

    msg->pose.pose.position.x/y --> pos at CoG
      see old version f1tenth_gym: src/racecar.cpp#L457,458
      BUT the scan pos is at Laser
      see old version f1tenth_gym: src/racecar.cpp#L410,411

  IN reality:
    to be verified!!!

  In ACADO Extended-Kinematic:
    omega is the angular velocity related to the
    "instantaneous center of curvature (see page 6 of Race Car vehicle Dynamics)"
    x,y vx,vy are at CoG and they are the notions in inertial system
  */
  ego_odom_time_stamp = msg->header.stamp;
  ego_x_(0) = msg->pose.pose.position.x; //x
  ego_x_(1) = msg->pose.pose.position.y; //y
  ego_x_(2) = tf::getYaw(msg->pose.pose.orientation); //yaw


  // SPEED and DELTA is precise when using /odom data directly from simulator
  double speed_x = msg->twist.twist.linear.x; //speed x, attention on which frame! (child or parent)
  double speed_y = msg->twist.twist.linear.y; //speed y, attention on which frame! (child or parent)
  double speed = sqrt((speed_x*speed_x)+(speed_y*speed_y)); //speed of the vehicle
  ego_x_(3) = speed;
  ego_x_(4) = msg->twist.twist.angular.z;

  if (who_am_i==0) { //ego
    // SPEED and DELTA is not provided from particle filter (at least in original version)
    // we use the sent command as the estimation
    ego_x_(3) = curr_cmd(0); //speed
    if (ego_x_(3)<0.5) ego_x_(3) = 0.5;
    ego_x_(4) = curr_cmd(1); //delta (not omega!!!)
  } else {
    if (ego_x_(3)<0.5) ego_x_(3) = 0.5;
  }

  ROS_INFO("---------------- x: %f, y: %f, yaw: %f, v: %f, omega: %f",
                            ego_x_(0), ego_x_(1), ego_x_(2), ego_x_(3), ego_x_(4));
  transform_ego_odom_to_relative_pos(ego_x_, ego_ex_);
  ego_ex_snapshot_ = ego_ex_;

  ROS_INFO("BEFORE  DC     --> ey %.2f, ephi %.2f, vx: %.2f, vy: %.2f, omega: %.2f, delta: %.2f, s: %.2f",
                            ego_ex_(0), ego_ex_(1), ego_ex_(2), ego_ex_(3), ego_ex_(4), ego_ex_(5), ego_ex_(7));

  if (who_am_i==1)
  {
    ros::TimerEvent t_virtual;
    updateCmd(t_virtual);
  } else {
    check_mode();
  }
  //ROS_DEBUG("    @ time     %f", (ros::Time::now()-init_event.current_real).toSec());
  //ROS_DEBUG("    on address %d", std::this_thread::get_id());
}

void oppOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  opp_x_(0) = msg->pose.pose.position.x; //x
  opp_x_(1) = msg->pose.pose.position.y; //y
  opp_x_(2) = tf::getYaw(msg->pose.pose.orientation); //yaw
  double speed_x = msg->twist.twist.linear.x; //speed x, attention on which frame! (child or parent)
  double speed_y = msg->twist.twist.linear.y; //speed y, attention on which frame! (child or parent)
  double speed = sqrt((speed_x*speed_x)+(speed_y*speed_y)); //speed of the vehicle
  opp_x_(3) = speed;
  opp_x_(4) = msg->twist.twist.angular.z;

  //ROS_INFO("---------------- x: %f, y: %f, yaw: %f, v: %f, omega: %f",
  //                          opp_x_(0), opp_x_(1), opp_x_(2), opp_x_(3), opp_x_(4));
  transform_opp_odom_to_relative_pos(opp_x_, opp_ex_);
  opp_ex_snapshot_ = opp_ex_;
  //ROS_DEBUG("    @ time     %f", (ros::Time::now()-init_event.current_real).toSec());
  //ROS_DEBUG("    on address %d", std::this_thread::get_id());
}

void updateCmd(const ros::TimerEvent& event) {
  std::chrono::duration<double, std::milli> dt_ms;
  // Get starting timepoint
  auto start = high_resolution_clock::now();

  auto t_start_millis = std::chrono::duration_cast<std::chrono::microseconds>(start.time_since_epoch()).count();
  double t_start_ms = (t_start_millis % 1000000) / 1000.0;
  ROS_INFO("--- start of nmpc ...: %7.3f [ms]", t_start_ms);

  ego_ex_snapshot_ = ego_ex_;
  opp_ex_snapshot_ = opp_ex_;

  double dt = (ros::Time::now()-ego_odom_time_stamp).toSec();
  if (dt>0.1) dt = 0.1;
  dt = 0;
  delay_compensation(ego_ex_snapshot_, dt);

  Eigen::VectorXd tmp_ego_pos(3);
  Eigen::VectorXd tmp_ego_state(3);
  tmp_ego_state[0] = ego_ex_snapshot_(7);
  tmp_ego_state[1] = ego_ex_snapshot_(0);
  tmp_ego_state[2] = ego_ex_snapshot_(1);
  transform_to_absolute_pos(tmp_ego_pos, tmp_ego_state);
  ROS_INFO("delay_compensation: x - %6.1f, y - %6.1f, theta - %6.1f", tmp_ego_pos[0], tmp_ego_pos[1], tmp_ego_pos[2]);
  ROS_INFO("prev_cmd: %f, %f", prev_cmd(0), prev_cmd(1));
  ROS_INFO("curr_cmd: %f, %f", curr_cmd(0), curr_cmd(1));

  if (who_am_i==0) {
    geometry_msgs::Quaternion q;
    q.w = cos(tmp_ego_pos[2] * 0.5);
    q.z = sin(tmp_ego_pos[2] * 0.5);
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = tmp_ego_pos[0];
    pose.pose.position.y = tmp_ego_pos[1];
    pose.pose.orientation = q;
    compensated_pose_pub.publish(pose);
  }

  //ROS_DEBUG("+++ updateCmd");
  //ROS_DEBUG("\n    @ %f", (event.current_real-init_event.current_real).toSec());
  //ROS_DEBUG("    on adre %d", std::this_thread::get_id());

  for (int i = 0; i < N; i++) {
    acadoVariables_opp.x[i*NX + 7] = opp_ex_snapshot_[7] + i*STEP_SIZE;
    acadoVariables_opp.x[i*NX + 0] = opp_ex_snapshot_[0];
  }
/*
#ifdef TEST_WCET
  for (int i = 0; i < N; i++) {
    acadoVariables_opp.x[i*NX + 7] = opp_ex_snapshot_[7] + i*STEP_SIZE;
    acadoVariables_opp.x[i*NX + 0] = opp_ex_snapshot_[0];
  }
#else
  acado_initializeSolver();

  //bool solvable = false; //qp_solver(opp_ex_snapshot_);
  bool solvable = qp_solver(opp_ex_snapshot_);
  // save opp optimal traj
  if (!solvable) {
    //ROS_WARN("opp INFEASIBLE!");
    for (int i = 0; i < N; i++) {
      acadoVariables_opp.x[i*NX + 7] = opp_ex_snapshot_[7] + i*STEP_SIZE;
      acadoVariables_opp.x[i*NX + 0] = opp_ex_snapshot_[0];
    }
  } else {
    acadoVariables_opp = acadoVariables;
  }
#endif
*/

  bool LO_solvable = false;
  bool RO_solvable = false;
  bool FO_solvable = false;
  bool FW_solvable = false;

  if (opp_x_(0) == 1e+6) {
    // FW
    func_FW(FW_solvable);
    ROS_WARN("FW: %d", FW_solvable);

    if (FW_solvable) {
      if (!TEST_WCET) visu_traj();
      state_type x = {
        acadoVariables.x0[0],
        acadoVariables.x0[1],
        acadoVariables.x0[2],
        acadoVariables.x0[3],
        acadoVariables.x0[4],
        acadoVariables.x0[5],
        acadoVariables.x0[6],
        acadoVariables.x0[7],
        acadoVariables.u[0],
        acadoVariables.u[1],
        acadoVariables.od[0]
      };
      integrate_in_term_of_t(x, 0);// qp_calc_period);

      prev_cmd = curr_cmd;
      curr_cmd(0) = x[2]; //sqrt(x[2]*x[2]+x[3]*x[3]);
      curr_cmd(1) = x[5];
      curr_cmd(2) = acadoVariables.u[0];
      curr_cmd(3) = acadoVariables.u[1];
    } else {
      prev_cmd = curr_cmd;
      for (int i = 0; i<4; i++)
        curr_cmd(i) = 0.0;
    }
  } else {
  // Left or Right
  check_mode();

  auto te = std::chrono::high_resolution_clock::now();
  dt_ms = te - start;
  PREP_time = dt_ms.count();

  // 2-thread in parallel not working...
  // reason found: QProblem init failed (line 45 in acado_*_qpoases_interface.cpp)
  // possiblely because of "Currently usage of separate instances of QProblem in different threads is not safe because it uses global shared message handler"
  // see: https://projects.coin-or.org/qpOASES/ticket/35
  #ifdef TEST_WCET
  func_FW(FW_solvable);
  func_LO(LO_solvable);
  func_RO(RO_solvable);
  #else
  std::thread th_LO(func_LO, std::ref(LO_solvable));
  std::thread th_RO(func_RO, std::ref(RO_solvable));
  th_LO.join();
  th_RO.join();
  #endif

  auto ts = std::chrono::high_resolution_clock::now();
  ROS_WARN("LO: %d", LO_solvable);
  ROS_WARN("RO: %d", RO_solvable);

  if (LO_solvable && RO_solvable) {
    if (acadoVariables_LO.x[(N-1)*NX+6] < acadoVariables_RO.x[(N-1)*NX+6])
      acadoVariables = acadoVariables_LO;
    else
      acadoVariables = acadoVariables_RO;
  } else if (!LO_solvable && RO_solvable) {
    acadoVariables = acadoVariables_RO;
  } else if (LO_solvable && !RO_solvable) {
    acadoVariables = acadoVariables_LO;
  }

  if (LO_solvable || RO_solvable) {
    if (!TEST_WCET) visu_traj();
    state_type x = {
      acadoVariables.x0[0],
      acadoVariables.x0[1],
      acadoVariables.x0[2],
      acadoVariables.x0[3],
      acadoVariables.x0[4],
      acadoVariables.x0[5],
      acadoVariables.x0[6],
      acadoVariables.x0[7],
      acadoVariables.u[0],
      acadoVariables.u[1],
      acadoVariables.od[0]
    };
    integrate_in_term_of_t(x, 0); // qp_calc_period);

    prev_cmd = curr_cmd;
    curr_cmd(0) = x[2]; //sqrt(x[2]*x[2]+x[3]*x[3]);
    curr_cmd(1) = x[5];
    curr_cmd(2) = acadoVariables.u[0];
    curr_cmd(3) = acadoVariables.u[1];
/*
    ROS_DEBUG("v: %f", curr_cmd(0));
    ROS_DEBUG("delta: %f", curr_cmd(1));
    ROS_WARN("DT === %f", acadoVariables.x[8 + 6]-acadoVariables.x0[6]);

    ROS_DEBUG("[ CURRENT ]   ==> ey %.2f, ephi %.2f, vx: %.2f, vy: %.2f, omega: %.2f, delta: %.2f, s: %.2f",
      acadoVariables.x0[0], acadoVariables.x0[1], acadoVariables.x0[2], acadoVariables.x0[3], acadoVariables.x0[4], acadoVariables.x0[5], acadoVariables.x0[7]);
    ROS_DEBUG("[ PREDICT ]   ==> ey %.2f, ephi %.2f, vx: %.2f, vy: %.2f, omega: %.2f, delta: %.2f, s: %.2f",
      x[0], x[1], x[2], x[3], x[4], x[5], x[7]);
*/
    te = std::chrono::high_resolution_clock::now();
    dt_ms = te - ts;
    AFTER_time = dt_ms.count();
    FO_time = 0;
  } else {
    prev_cmd = curr_cmd;
    te = std::chrono::high_resolution_clock::now();
    dt_ms = te - ts;
    AFTER_time = dt_ms.count();

    ts = std::chrono::high_resolution_clock::now();

    double dist_s = abs(opp_ex_snapshot_(7) - ego_ex_snapshot_(7));
    double dist_ey = abs(opp_ex_snapshot_(0) - ego_ex_snapshot_(0));

    func_FO(FO_solvable);
    if (!FO_solvable || dist_s<0.3 || (dist_s<0.6&&dist_ey<0.1))
      for (int i = 0; i<4; i++)
        curr_cmd(i) = 0.0;
    te = std::chrono::high_resolution_clock::now();
    dt_ms = te - ts;
    FO_time = dt_ms.count();
  }
  }

  if (TEST_WCET) return;

  ackermann_msgs::AckermannDriveStamped drive_msg;
  drive_msg.drive.speed = curr_cmd(0);
  drive_msg.drive.steering_angle = curr_cmd(1);
  cmd_pub.publish(drive_msg);
/*
  ROS_WARN("v: %f", curr_cmd(0));
  ROS_WARN("delta: %f", curr_cmd(1));
  ROS_WARN("a: %f", curr_cmd(2));
  ROS_WARN("vdelta: %f", curr_cmd(3));
  ROS_WARN("");
*/
  // Get ending timepoint
  auto stop = high_resolution_clock::now();
  duration<double, std::milli> fp_ms = stop - start;
  //ROS_WARN("CALC TIME: %.2f [ms] - AD TIME: %.2f [ms]", fp_ms, (acadoVariables.x[NX+6]-acadoVariables.x[6])*1000);
/*
  ROS_WARN("LO - timing: %f", LO_time_counter/LO_iter_counter);
  ROS_WARN("        max: %f", LO_time_max);
  ROS_WARN("        min: %f", LO_time_min);
  ROS_WARN("RO - timing: %f", RO_time_counter/RO_iter_counter);
  ROS_WARN("        max: %f", RO_time_max);
  ROS_WARN("        min: %f", RO_time_min);
  ROS_WARN("FO - timing: %f", FO_time_counter/FO_iter_counter);
  ROS_WARN("        max: %f", FO_time_max);
  ROS_WARN("        min: %f", FO_time_min);
*/

  file_trace_ << start.time_since_epoch().count() << ", " << stop.time_since_epoch().count() << std::endl;

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_end_millis = std::chrono::duration_cast<std::chrono::microseconds>(t_end.time_since_epoch()).count();
  double t_end_ms = (t_end_millis % 1000000) / 1000.0;
  ROS_INFO("--- end of nmpc ...: %7.3f [ms]\n", t_end_ms);
}
