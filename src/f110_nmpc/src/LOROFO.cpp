#include "nmpc_ctrl.hpp"

extern std::ofstream file_;
extern std::ofstream file_trace_;
extern std::ofstream file_trace_LO_FW_;
extern std::ofstream file_trace_RO_;
extern std::ofstream file_trace_FO_;
extern std::chrono::time_point<std::chrono::high_resolution_clock> chrono_init_time;

bool calibration_lo_fw = false;
bool calibration_ro = false;

bool sum_LO_solvable;
bool sum_RO_solvable;
bool sum_FO_solvable;
bool sum_FW_solvable;

double curr_cmd_0_FO;
double curr_cmd_1_FO;
double curr_cmd_2_FO;
double curr_cmd_3_FO;

//void calcLOFW(const ros::TimerEvent& event) {
//void calcLOFW(const std_msgs::Bool::ConstPtr& msg) {
void calcLOFW(const nav_msgs::Odometry::ConstPtr& msg) {
  release_ctrl();

  // trigger RO
  std::thread th_RO(calcRO);
  th_RO.join();

  // start LO, FO
  egoOdomCallback(msg);

  auto start = high_resolution_clock::now();
  auto t_start_millis = std::chrono::duration_cast<std::chrono::microseconds>(start.time_since_epoch()).count();
  double t_start_ms = (t_start_millis % 1000000) / 1000.0;
  ROS_INFO("--- start of nmpc LO/FW ...: %7.3f [ms]", t_start_ms);

  ego_ex_snapshot_LO = ego_ex_;
  opp_ex_snapshot_LO = opp_ex_;

  double dt = (ros::Time::now()-ego_odom_time_stamp).toSec();
  if (dt>0.1) dt = 0.1;
  delay_compensation(ego_ex_snapshot_LO, dt);

  Eigen::VectorXd tmp_ego_pos(3);
  Eigen::VectorXd tmp_ego_state(3);
  tmp_ego_state[0] = ego_ex_snapshot_LO(7);
  tmp_ego_state[1] = ego_ex_snapshot_LO(0);
  tmp_ego_state[2] = ego_ex_snapshot_LO(1);
  transform_to_absolute_pos(tmp_ego_pos, tmp_ego_state);

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

  for (int i = 0; i < N; i++) {
    acadoVariables_opp.x[i*NX + 7] = opp_ex_snapshot_LO[7] + i*STEP_SIZE;
    acadoVariables_opp.x[i*NX + 0] = opp_ex_snapshot_LO[0];
  }

  bool LO_solvable = false;
  bool FW_solvable = false;
  sum_FW_solvable = false;

  if (opp_x_(0) == 1e+6) {
    // FW
    func_FW(FW_solvable);
    ROS_WARN("FW: %d", FW_solvable);

    if (FW_solvable) {
      visu_traj();
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
      integrate_in_term_of_t(x, qp_calc_period);

      // FW
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
    sum_FW_solvable = true;
    sum_LO_solvable = false;
  } else {
    func_LO(LO_solvable);
    sum_LO_solvable = LO_solvable;
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_end_millis = std::chrono::duration_cast<std::chrono::microseconds>(t_end.time_since_epoch()).count();
  double t_end_ms = (t_end_millis % 1000000) / 1000.0;
  ROS_INFO("--- end of nmpc LO/FW ...: %7.3f [ms]\n", t_end_ms);

  file_trace_LO_FW_ << start.time_since_epoch().count() << ", " << t_end.time_since_epoch().count() << std::endl;

  //calcRO();
  calcFO();
  //release_ctrl();
  /*
  std_msgs::Bool msg_result;
  msg_result.data = true;
  pub_ctrler_complete.publish(msg_result);
  */
}

//void calcRO(const ros::TimerEvent& event) {
//void calcRO(const std_msgs::Bool::ConstPtr& msg) {
void calcRO() {
  auto start = high_resolution_clock::now();
  auto t_start_millis = std::chrono::duration_cast<std::chrono::microseconds>(start.time_since_epoch()).count();
  double t_start_ms = (t_start_millis % 1000000) / 1000.0;
  ROS_INFO("--- start of nmpc RO ...: %7.3f [ms]", t_start_ms);

  if (opp_x_(0) != 1e+6) {
    ego_ex_snapshot_RO = ego_ex_;
    opp_ex_snapshot_RO = opp_ex_;

    double dt = (ros::Time::now()-ego_odom_time_stamp).toSec();
    if (dt>0.1) dt = 0.1;
    delay_compensation(ego_ex_snapshot_RO, dt);

    for (int i = 0; i < N; i++) {
      acadoVariables_opp.x[i*NX + 7] = opp_ex_snapshot_RO[7] + i*STEP_SIZE;
      acadoVariables_opp.x[i*NX + 0] = opp_ex_snapshot_RO[0];
    }

    bool RO_solvable = false;
    func_RO(RO_solvable);
    sum_RO_solvable = RO_solvable;
  } else {
    sum_RO_solvable = false;
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_end_millis = std::chrono::duration_cast<std::chrono::microseconds>(t_end.time_since_epoch()).count();
  double t_end_ms = (t_end_millis % 1000000) / 1000.0;
  ROS_INFO("--- end of nmpc RO ...: %7.3f [ms]\n", t_end_ms);

  file_trace_RO_ << start.time_since_epoch().count() << ", " << t_end.time_since_epoch().count() << std::endl;
}

//void calcFO(const std_msgs::Bool::ConstPtr& msg) {
void calcFO() {
  auto start = high_resolution_clock::now();
  auto t_start_millis = std::chrono::duration_cast<std::chrono::microseconds>(start.time_since_epoch()).count();
  double t_start_ms = (t_start_millis % 1000000) / 1000.0;
  ROS_INFO("--- start of nmpc FO ...: %7.3f [ms]", t_start_ms);

  if (opp_x_(0) != 1e+6) {
    ego_ex_snapshot_FO = ego_ex_;
    opp_ex_snapshot_FO = opp_ex_;

    bool FO_solvable = false;
    func_FO(FO_solvable);
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_end_millis = std::chrono::duration_cast<std::chrono::microseconds>(t_end.time_since_epoch()).count();
  double t_end_ms = (t_end_millis % 1000000) / 1000.0;
  ROS_INFO("--- end of nmpc FO ...: %7.3f [ms]\n", t_end_ms);

  file_trace_FO_ << start.time_since_epoch().count() << ", " << t_end.time_since_epoch().count() << std::endl;
}

//void release_ctrl(const std_msgs::Bool::ConstPtr& msg) {
void release_ctrl() {
  ROS_WARN("LO: %d", sum_LO_solvable);
  ROS_WARN("RO: %d", sum_RO_solvable);
  ROS_WARN("FO: %d", sum_FO_solvable);
  ROS_WARN("FW: %d", sum_FW_solvable);

  if (!sum_FW_solvable) {
    if (sum_LO_solvable && sum_RO_solvable) {
      if (acadoVariables_LO.x[(N-1)*NX+6] < acadoVariables_RO.x[(N-1)*NX+6])
        acadoVariables = acadoVariables_LO;
      else
        acadoVariables = acadoVariables_RO;
    } else if (!sum_LO_solvable && sum_RO_solvable) {
      acadoVariables = acadoVariables_RO;
    } else if (sum_LO_solvable && !sum_RO_solvable) {
      acadoVariables = acadoVariables_LO;
    }

    if (sum_LO_solvable || sum_RO_solvable) {
      visu_traj();
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
      integrate_in_term_of_t(x, qp_calc_period);

      // LO or RO
      prev_cmd = curr_cmd;
      curr_cmd(0) = x[2];
      curr_cmd(1) = x[5];
      curr_cmd(2) = acadoVariables.u[0];
      curr_cmd(3) = acadoVariables.u[1];

    } else {

      double dist_s = abs(opp_ex_snapshot_(7) - ego_ex_snapshot_(7));
      double dist_ey = abs(opp_ex_snapshot_(0) - ego_ex_snapshot_(0));

      if (!sum_FO_solvable || dist_s<0.3 || (dist_s<0.6&&dist_ey<0.1)) {
        // Braking
        for (int i = 0; i<4; i++)
          curr_cmd(i) = 0.0;
      } else {
        // FO
        curr_cmd(0) = curr_cmd_0_FO;
        curr_cmd(1) = curr_cmd_1_FO;
        curr_cmd(2) = curr_cmd_2_FO;
        curr_cmd(3) = curr_cmd_3_FO;
      }

    }
  }

  ackermann_msgs::AckermannDriveStamped drive_msg;
  drive_msg.drive.speed = curr_cmd(0);
  drive_msg.drive.steering_angle = curr_cmd(1);
  cmd_pub.publish(drive_msg);
}
