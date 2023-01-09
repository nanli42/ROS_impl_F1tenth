#include "nmpc_ctrl.hpp"
#define TEST_WCET 0

extern double LO_time;
extern double RO_time;
extern double FW_time;
extern int LO_iter_num;
extern int RO_iter_num;
extern int FW_iter_num;
extern double LO_kkt_val;
extern double RO_kkt_val;
extern double FW_kkt_val;

extern double curr_cmd_0_FO;
extern double curr_cmd_1_FO;
extern double curr_cmd_2_FO;
extern double curr_cmd_3_FO;

void func_FW(bool& result) {
  auto ts = std::chrono::high_resolution_clock::now();
    acado_initializeSolver();
    if (who_am_i == 0)
      result = qp_solver(ego_ex_snapshot_LO);
    else
      result = qp_solver(ego_ex_snapshot_);
  auto te = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> dt_ms = te - ts;
  FW_time = dt_ms.count();
}


void func_LO(bool& result) {
  auto ts = std::chrono::high_resolution_clock::now();
  if (who_am_i == 0)
    result = qp_solver_LO(ego_ex_snapshot_LO);
  else
    result = qp_solver_LO(ego_ex_snapshot_);
  auto te = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> dt_ms = te - ts;
  LO_time = dt_ms.count();
  //ROS_DEBUG("ID: %d - result %d", std::this_thread::get_id(), result);
}

void func_RO(bool& result) {
  auto ts = std::chrono::high_resolution_clock::now();
  if (who_am_i == 0)
    result = qp_solver_RO(ego_ex_snapshot_RO);
  else
    result = qp_solver_RO(ego_ex_snapshot_);
  auto te = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> dt_ms = te - ts;
  RO_time = dt_ms.count();
  //ROS_DEBUG("ID: %d - result %d", std::this_thread::get_id(), result);
}

void func_FO(bool& result) {
  if (who_am_i == 1) ego_ex_snapshot_FO = ego_ex_snapshot_;

  auto start = high_resolution_clock::now();

  // if too close, exit false
  if ( abs(ego_ex_snapshot_FO[7]-opp_ex_snapshot_FO[7]) < STEP_SIZE ) {
    result = false;
    return;
  }

  // calc velocity
  double v = 0.0;
  if (abs(ego_ex_snapshot_FO[7]-opp_ex_snapshot_FO[7]) > STEP_SIZE*3)
    v = sqrt(opp_ex_snapshot_FO[2]*opp_ex_snapshot_FO[2]+opp_ex_snapshot_FO[3]*opp_ex_snapshot_FO[3]);
  else if (abs(ego_ex_snapshot_FO[7]-opp_ex_snapshot_FO[7]) > STEP_SIZE*2)
    v = 1.0;
  else
    v = 0.5;

  // calc delta
  double L = 0.3302;

  Eigen::VectorXd ego_pos(3);
  Eigen::VectorXd ego_state(3);
  ego_state[0] = ego_ex_snapshot_FO(7);
  ego_state[1] = ego_ex_snapshot_FO(0);
  ego_state[2] = ego_ex_snapshot_FO(1);
  transform_to_absolute_pos(ego_pos, ego_state);
  Eigen::VectorXd opp_pos(3);
  Eigen::VectorXd opp_state(3);
  opp_state[0] = opp_ex_snapshot_FO(7);
  opp_state[1] = opp_ex_snapshot_FO(0);
  opp_state[2] = opp_ex_snapshot_FO(1);
  transform_to_absolute_pos(opp_pos, opp_state);

  double ld = sqrt(
    (ego_pos(0)-opp_pos(0))*(ego_pos(0)-opp_pos(0))
  + (ego_pos(1)-opp_pos(1))*(ego_pos(1)-opp_pos(1))
  );

  double alpha = atan2(opp_pos(1)-ego_pos(1), opp_pos(0)-ego_pos(0)) - ego_pos(2);

  double delta = atan(2*L*sin(alpha)/ld);
  if (delta > +0.41) delta = +0.41;
  if (delta < -0.41) delta = -0.41;

  curr_cmd_0_FO = v;
  curr_cmd_1_FO = delta;
  curr_cmd_2_FO = 0;
  curr_cmd_3_FO = (delta-curr_cmd(1))/qp_calc_period;

  result = true;

  auto stop = high_resolution_clock::now();
  duration<double, std::milli> fp_ms = stop - start;
  FO_iter_counter++;
  FO_time_counter += fp_ms.count();

  if (fp_ms.count()>FO_time_max) FO_time_max = fp_ms.count();
  if (fp_ms.count()<FO_time_min) FO_time_min = fp_ms.count();
  if (fp_ms.count()>FO_time_max) FO_time_max = fp_ms.count();
  if (fp_ms.count()<FO_time_min) FO_time_min = fp_ms.count();
}

void set_constraint_LO() {
  if (!RESP) return;
  if (who_am_i==1) ego_ex_snapshot_LO = ego_ex_snapshot_;

  double opp_s_0 = acadoVariables_opp.x[ 0*NX + 7 ];

  if (opp_s_0-ego_ex_snapshot_LO[7]>5*STEP_SIZE) return;

  int idx = -1;
  for (int i = 0; i < N; i++) {
    double ego_s = ego_ex_snapshot_LO[7] + i*STEP_SIZE;
    if ( (ego_s > opp_s_0) || (ego_s <= opp_s_0 && ego_s+STEP_SIZE > opp_s_0) ) {
      idx = i;
      break;
    }
  }

  if (idx<0) idx = 0;
  //ROS_DEBUG("idx: %d [L]", idx);

  // ego idx - opp 0, idx+1 - 1, idx+2 - 2, idx+3 - 3

  int M = 3;
  double safe_ey_dist = 0.6; //sqrt( lr*lr + lf*lf ) * 2.5; // bubble radius * 2
  double lrf = sqrt( lr*lr + lf*lf );
  for (int i = 0; i <= M && i < N && i+idx < N; i++) {
    acadoVariables_LO.lbAValues[(i+idx)*8+0] = acadoVariables_opp.x[i*8+0] + safe_ey_dist;
    //ROS_DEBUG("lbAValues: %f", acadoVariables.lbAValues[(i+idx)*8+0]);
  }
}

void set_constraint_RO() {
  if (!RESP) return;
  if (who_am_i==1) ego_ex_snapshot_RO = ego_ex_snapshot_;

  double opp_s_0 = acadoVariables_opp.x[ 0*NX + 7 ];

  if (opp_s_0-ego_ex_snapshot_RO[7]>5*STEP_SIZE) return;

  int idx = -1;
  for (int i = 0; i < N; i++) {
    double ego_s = ego_ex_snapshot_RO[7] + i*STEP_SIZE;
    if ( (ego_s > opp_s_0) || (ego_s <= opp_s_0 && ego_s+STEP_SIZE > opp_s_0) ) {
      idx = i;
      break;
    }
  }

  if (idx<0) idx = 0;
  //ROS_DEBUG("idx: %d [R]", idx);

  // ego idx - opp 0, idx+1 - 1, idx+2 - 2, idx+3 - 3

  int M = 3;
  double safe_ey_dist = 0.6; //sqrt( lr*lr + lf*lf ) * 2.5; // bubble radius * 2
  double lrf = sqrt( lr*lr + lf*lf );
  for (int i = 0; i <= M && i < N && i+idx < N; i++) {
    acadoVariables_RO.ubAValues[(i+idx)*8+0] = acadoVariables_opp.x[i*8+0] - safe_ey_dist;
    //ROS_DEBUG("ubAValues: %f", acadoVariables.ubAValues[(i+idx)*8+0]);
  }
}

bool qp_solver(Eigen::VectorXd init_state) {
  //ros::Time qp_init_time = ros::Time::now();
  FW_iter_num = 0;
  FW_kkt_val = -1;

  //acado_initializeSolver();

  acado_timer t;
  acado_tic( &t );

  for (int i = 0; i <= N; ++i)  {
    double s = init_state[7] + i*STEP_SIZE;
    acadoVariables.od[ i ] = get_curature(s);
  }

  /* Initialize the states and controls. */
  for (int i = 0; i < N; ++i)  {
    acadoVariables.u[ i*NU + 0 ] = 0.0;
    acadoVariables.u[ i*NU + 1 ] = 0.0;
  }
  for (int i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
  for (int i = 0; i <= N; ++i)  {
    acadoVariables.x[ i*NX + 2 ] = 2.0;

    acadoVariables.x[ i*NX + 5 ] = atan(acadoVariables.od[ i ]*(lr+lf));
    if (acadoVariables.x[ i*NX + 5 ]>0.41) acadoVariables.x[ i*NX + 5 ] = 0.41;
    if (acadoVariables.x[ i*NX + 5 ]<-0.41) acadoVariables.x[ i*NX + 5 ] = -0.41;

    acadoVariables.x[ i*NX + 4 ] = acadoVariables.x[ i*NX + 5 ]*acadoVariables.x[ i*NX + 2 ]/(lr+lf);
    acadoVariables.x[ i*NX + 3 ] = acadoVariables.x[ i*NX + 4 ]*lr;

    acadoVariables.x[ i*NX + 7 ] = init_state[7] + i*STEP_SIZE;
  }


  acadoVariables.x0[ 0 ] = init_state[0];
  acadoVariables.x0[ 1 ] = init_state[1];
  acadoVariables.x0[ 2 ] = init_state[2];
  if (init_state[2]<0.1) acadoVariables.x0[ 2 ] = 0.1;
  acadoVariables.x0[ 3 ] = init_state[3];
  acadoVariables.x0[ 4 ] = init_state[4];
  acadoVariables.x0[ 5 ] = init_state[5];
  acadoVariables.x0[ 6 ] = init_state[6];
  acadoVariables.x0[ 7 ] = init_state[7];

  double prec = 1e+6;
  int iter_step = 0;
  while (prec>1e-3&&iter_step<10) {
    acado_preparationStep();
    acado_feedbackStep();
    prec = acado_getKKT();
    ROS_WARN("acaodo_getKKT(): %f", acado_getKKT());
    if (prec>1e+6||isnan(prec)||prec==0) {
      for (int i = 0; i <= N; i++) {
        acadoVariables.x[ i*NX + 0 ] = acadoVariables.x0[ 0 ];
        acadoVariables.x[ i*NX + 7 ] = init_state[7] + i*STEP_SIZE;
      }
      iter_step++;
      FW_iter_num = iter_step;
      return false;
    }
    /*
    if ((qp_init_time-ros::Time::now()).toSec()>qp_calc_period) {
      std::cout << "\n\n\nTIME OUT!\n\n\n" << std::endl;
    }
    */
    iter_step++;
  }
  real_t te = acado_toc( &t );
  //ROS_DEBUG("acaodo_getKKT(): %f", acado_getKKT());
  //ROS_WARN("timing: %f", te*1e+3);

  FW_iter_num = iter_step;
  FW_kkt_val = acado_getKKT();

  if (prec>1e-2) return false;
  else
  return true;
}

bool qp_solver_LO(Eigen::VectorXd init_state0) {
  Eigen::VectorXd init_state = init_state0;
  LO_iter_num = 0;
  LO_kkt_val = -1;

  ros::Time qp_init_time = ros::Time::now();

  acado_LO_timer t;
  acado_LO_tic( &t );

  acado_LO_initializeSolver();
  set_constraint_LO();

  for (int i = 0; i <= N; ++i)  {
    double s = init_state[7] + i*STEP_SIZE;
    acadoVariables_LO.od[ i ] = get_curature(s);
  }

  /* Initialize the states and controls. */
  for (int i = 0; i < N; ++i)  {
    acadoVariables_LO.u[ i*NU + 0 ] = 0.0;
    acadoVariables_LO.u[ i*NU + 1 ] = 0.0;
  }
  for (int i = 0; i < NX * (N + 1); ++i)  acadoVariables_LO.x[ i ] = 0.0;
  for (int i = 0; i <= N; ++i)  {
    acadoVariables_LO.x[ i*NX + 2 ] = 2.0;

    acadoVariables_LO.x[ i*NX + 5 ] = atan(acadoVariables_LO.od[ i ]*(lr+lf));
    if (acadoVariables_LO.x[ i*NX + 5 ]>0.41) acadoVariables_LO.x[ i*NX + 5 ] = 0.41;
    if (acadoVariables_LO.x[ i*NX + 5 ]<-0.41) acadoVariables_LO.x[ i*NX + 5 ] = -0.41;

    acadoVariables_LO.x[ i*NX + 4 ] = acadoVariables_LO.x[ i*NX + 5 ]*acadoVariables_LO.x[ i*NX + 2 ]/(lr+lf);
    acadoVariables_LO.x[ i*NX + 3 ] = acadoVariables_LO.x[ i*NX + 4 ]*lr;

    acadoVariables_LO.x[ i*NX + 7 ] = init_state[7] + i*STEP_SIZE;
  }


  acadoVariables_LO.x0[ 0 ] = init_state[0];
  acadoVariables_LO.x0[ 1 ] = init_state[1];
  acadoVariables_LO.x0[ 2 ] = init_state[2];
  if (init_state[2]<0.1) acadoVariables_LO.x0[ 2 ] = 0.1;
  acadoVariables_LO.x0[ 3 ] = init_state[3];
  acadoVariables_LO.x0[ 4 ] = init_state[4];
  acadoVariables_LO.x0[ 5 ] = init_state[5];
  acadoVariables_LO.x0[ 6 ] = init_state[6];
  acadoVariables_LO.x0[ 7 ] = init_state[7];

  double prec = 1e+6;
  int iter_step = 0;
  while (prec>1e-3&&iter_step<10) {
    acado_LO_preparationStep();
    acado_LO_feedbackStep();
    prec = acado_LO_getKKT();
    //ROS_DEBUG("=== (LO) KKT(): %f", prec);
    if (prec>1e+6||isnan(prec)||prec==0) {
      for (int i = 0; i <= N; i++) {
        acadoVariables_LO.x[ i*NX + 0 ] = acadoVariables_LO.x0[ 0 ];
        acadoVariables_LO.x[ i*NX + 7 ] = init_state[7] + i*STEP_SIZE;
      }
      iter_step++;
      LO_iter_num = iter_step;
      return false;
    }
    if (!TEST_WCET) {
      if ((qp_init_time-ros::Time::now()).toSec()>qp_calc_period) {
        std::cout << "\n\n\nTIME OUT!\n\n\n" << std::endl;
        real_t te = acado_LO_toc( &t );
        //ROS_DEBUG("acaodo_getKKT(): %f", acado_LO_getKKT());
        LO_iter_counter++;
        LO_time_counter += te*1e+3;
        if (te*1e+3>LO_time_max) LO_time_max = te*1e+3;
        if (te*1e+3<LO_time_min) LO_time_min = te*1e+3;
        iter_step++;
        LO_iter_num = iter_step;
        return false;
      }
    }
    iter_step++;
  }
  real_t te = acado_LO_toc( &t );
  //std::cout << "it. in LO: " << iter_step << std::endl;
  //ROS_INFO("acaodo_getKKT(): %f", acado_LO_getKKT());
  //ROS_INFO("              t: %f", te*1e+3);
  LO_iter_counter++;
  LO_time_counter += te*1e+3;
  if (te*1e+3>LO_time_max) LO_time_max = te*1e+3;
  if (te*1e+3<LO_time_min) LO_time_min = te*1e+3;

  LO_iter_num = iter_step;
  LO_kkt_val = acado_LO_getKKT();

  if (prec>1e-2) return false;
  else
  return true;
}

bool qp_solver_RO(Eigen::VectorXd init_state0) {
  Eigen::VectorXd init_state = init_state0;
  RO_iter_num = 0;
  RO_kkt_val = -1;

  ros::Time qp_init_time = ros::Time::now();

  acado_RO_timer t;
  acado_RO_tic( &t );

  acado_RO_initializeSolver();
  set_constraint_RO();

  for (int i = 0; i <= N; ++i)  {
    double s = init_state[7] + i*STEP_SIZE;
    acadoVariables_RO.od[ i ] = get_curature(s);
  }

  /* Initialize the states and controls. */
  for (int i = 0; i < N; ++i)  {
    acadoVariables_RO.u[ i*NU + 0 ] = 0.0;
    acadoVariables_RO.u[ i*NU + 1 ] = 0.0;
  }
  for (int i = 0; i < NX * (N + 1); ++i)  acadoVariables_RO.x[ i ] = 0.0;
  for (int i = 0; i <= N; ++i)  {
    acadoVariables_RO.x[ i*NX + 2 ] = 2.0;

    acadoVariables_RO.x[ i*NX + 5 ] = atan(acadoVariables_RO.od[ i ]*(lr+lf));
    if (acadoVariables_RO.x[ i*NX + 5 ]>0.41) acadoVariables_RO.x[ i*NX + 5 ] = 0.41;
    if (acadoVariables_RO.x[ i*NX + 5 ]<-0.41) acadoVariables_RO.x[ i*NX + 5 ] = -0.41;

    acadoVariables_RO.x[ i*NX + 4 ] = acadoVariables_RO.x[ i*NX + 5 ]*acadoVariables_RO.x[ i*NX + 2 ]/(lr+lf);
    acadoVariables_RO.x[ i*NX + 3 ] = acadoVariables_RO.x[ i*NX + 4 ]*lr;

    acadoVariables_RO.x[ i*NX + 7 ] = init_state[7] + i*STEP_SIZE;
  }


  acadoVariables_RO.x0[ 0 ] = init_state[0];
  acadoVariables_RO.x0[ 1 ] = init_state[1];
  acadoVariables_RO.x0[ 2 ] = init_state[2];
  if (init_state[2]<0.1) acadoVariables_RO.x0[ 2 ] = 0.1;
  acadoVariables_RO.x0[ 3 ] = init_state[3];
  acadoVariables_RO.x0[ 4 ] = init_state[4];
  acadoVariables_RO.x0[ 5 ] = init_state[5];
  acadoVariables_RO.x0[ 6 ] = init_state[6];
  acadoVariables_RO.x0[ 7 ] = init_state[7];

  double prec = 1e+6;
  int iter_step = 0;
  while (prec>1e-3&&iter_step<10) {
    acado_RO_preparationStep();
    acado_RO_feedbackStep();
    prec = acado_RO_getKKT();
    //ROS_WARN("=== (RO) KKT(): %f", prec);
    if (prec>1e+6||isnan(prec)||prec==0) {
      for (int i = 0; i <= N; i++) {
        acadoVariables_RO.x[ i*NX + 0 ] = acadoVariables_RO.x0[ 0 ];
        acadoVariables_RO.x[ i*NX + 7 ] = init_state[7] + i*STEP_SIZE;
      }
      iter_step++;
      RO_iter_num = iter_step;
      return false;
    }
    if (!TEST_WCET) {
      if ((qp_init_time-ros::Time::now()).toSec()>qp_calc_period) {
        std::cout << "\n\n\nTIME OUT!\n\n\n" << std::endl;
        real_t te = acado_RO_toc( &t );
        RO_iter_counter++;
        RO_time_counter += te*1e+3;
        if (te*1e+3>RO_time_max) RO_time_max = te*1e+3;
        if (te*1e+3<RO_time_min) RO_time_min = te*1e+3;
        iter_step++;
        RO_iter_num = iter_step;
        return false;
      }
    }
    iter_step++;
  }
  real_t te = acado_RO_toc( &t );
  //std::cout << "it. in RO: " << iter_step << std::endl;
  //ROS_INFO("acaodo_getKKT(): %f", acado_RO_getKKT());
  //ROS_INFO("              t: %f", te*1e+3);
  RO_iter_counter++;
  RO_time_counter += te*1e+3;
  if (te*1e+3>RO_time_max) RO_time_max = te*1e+3;
  if (te*1e+3<RO_time_min) RO_time_min = te*1e+3;

  RO_iter_num = iter_step;
  RO_kkt_val = acado_LO_getKKT();

  if (prec>1e-2) return false;
  else
  return true;
}

void visu_traj() {
  Eigen::MatrixXd plot_line_points(N+1,2);
  for (int i = 0; i < N+1; i++) {
    Eigen::VectorXd pos(3);
    Eigen::VectorXd state(3);
    state[0] = acadoVariables.x[i*NX + 7];
    state[1] = acadoVariables.x[i*NX + 0];
    state[2] = acadoVariables.x[i*NX + 1];
    transform_to_absolute_pos(pos, state);
    plot_line_points(i,0) = pos(0);
    plot_line_points(i,1) = pos(1);
  }
  show_trajectory_in_rviz(plot_line_points, 0.2, 0);
}
