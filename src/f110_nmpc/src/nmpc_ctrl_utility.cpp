#include "nmpc_ctrl.hpp"
#include "LUT_occ_areas.hpp"

using namespace boost::numeric::odeint;

void check_mode() {
    // mode switching
    state_machine_previous_mode = state_machine_current_mode;
    if (ego_ex_snapshot_[7]<opp_ex_snapshot_[7]) state_machine_current_mode = FOLLOWING_mode;
    else state_machine_current_mode = LEADING_mode;

    // state machine
    if (state_machine_previous_mode == LEADING_mode &&
      state_machine_current_mode == FOLLOWING_mode) {
        time_count_L2F = 10;
        RESP = false;
      }
    if (state_machine_previous_mode == FOLLOWING_mode &&
      state_machine_current_mode == FOLLOWING_mode) {
        if (time_count_L2F>0) {
          time_count_L2F--;
          RESP = false;
        } else {
          RESP = true;
        }
      }
    if (state_machine_previous_mode == FOLLOWING_mode &&
      state_machine_current_mode == LEADING_mode) {
        time_count_F2L = 10;
        RESP = true;
      }
    if (state_machine_previous_mode == LEADING_mode &&
      state_machine_current_mode == LEADING_mode) {
        if (time_count_F2L>0) {
          time_count_F2L--;
          RESP = true;
        } else {
          RESP = false;
        }
      }
}

double get_curature(double s) {
  double ss = s;
  if (ss>87.1) ss -= 87.1;
  double ks = 1e-6;
  for (int i = 0; i < 12; i++) {
    if (ss>=LUT[i][8]&&ss<LUT[i+1][8]) {
      ks = LUT[i][9];
      break;
    }
  }
  return ks;
}

void transform_ego_odom_to_relative_pos(Eigen::VectorXd & x_, Eigen::VectorXd & ex_) {
    double x = x_(0);
    double y = x_(1);
    double theta = x_(2);
    int box_nb = 12;

    for (int i = 0; i < 12; i++) {
      if ( (x>=LUT[i][0]&&y>=LUT[i][1]) && (x<=LUT[i][0]+LUT[i][2]&&y<=LUT[i][1]+LUT[i][3]) ) {
        box_nb = i;
        break;
      }
    }

    double s0 = LUT[box_nb][8];
    double x0 = LUT[box_nb][4];
    double y0 = LUT[box_nb][5];
    double x1 = LUT[box_nb][6];
    double y1 = LUT[box_nb][7];
    double PI = 3.14159;
    double s, ey, ephi;

    if (box_nb==0||box_nb==4||box_nb==6||box_nb==8||box_nb==10||box_nb==12) {
      // the case for LINE-type parts
      double ax = x-x0;
      double ay = y-y0;
      double bx = x1-x0;
      double by = y1-y0;

      double ab_dot = ax*bx + ay*by;
      double ab_cross = ax*by - ay*bx;
      double b_norm = sqrt(bx*bx + by*by);

      s = s0 + ab_dot/b_norm;

      double cx = ax-(ab_dot/b_norm)*(bx/b_norm);
      double cy = ay-(ab_dot/b_norm)*(by/b_norm);
      ey = sqrt(cx*cx + cy*cy);
      if (ab_cross>0) ey = -ey;

      ephi = theta - atan2(by,bx);
    } else {
      // the case for CIRCLE-type parts
      if (box_nb==2||box_nb==7) {
        // counter-clockwise cases
        double r0x = (x0+x1)/2;
        double r0y = (y0+y1)/2;
        double theta0 = atan2(y0-r0y, x0-r0x);
        double theta1 = atan2(y-r0y, x-r0x);

        double abs_dt = abs(theta1-theta0);
        if (abs_dt>PI) abs_dt = abs(abs_dt-PI*2);
        s = s0 + abs_dt*2.5;
        ey = -(sqrt((y-r0y)*(y-r0y)+(x-r0x)*(x-r0x))-2.5);
        ephi = theta - (theta1+PI/2);
      } else {
        // clockwise cases
        double r0x, r0y;
        if (box_nb==5) {
          r0x = x0;
          r0y = y1;
        } else
        if (box_nb==11) {
          r0x = x1;
          r0y = y0;
        }
        else {
          r0x = (x0+x1)/2;
          r0y = (y0+y1)/2;
        }
        double theta0 = atan2(y0-r0y, x0-r0x);
        double theta1 = atan2(y-r0y, x-r0x);

        double abs_dt = abs(theta0-theta1);
        if (abs_dt>PI) abs_dt = abs(abs_dt-PI*2);
        s = s0 + abs_dt*2.5;
        ey = sqrt((y-r0y)*(y-r0y)+(x-r0x)*(x-r0x))-2.5;
        ephi = theta - (theta1-PI/2);
      }
    }
    if (ephi>PI) ephi = ephi - PI*2;
    if (ephi<-PI) ephi = ephi + PI*2;

    ex_(0) = ey;
    ex_(1) = ephi;

    if (ex_(0) > 1.3) ex_(0) = 1.30;
    if (ex_(0) < -1.3) ex_(0) = -1.30;

    // for opp: use precise /odom data from simulator
    if (who_am_i==1) {
      // vy = omega*lr
      ex_(3) = x_(4)*lr;
      // omega
      ex_(4) = x_(4);

      // vx*vx = v*v - vy*vy
      if (x_(3)*x_(3)>ex_(3)*ex_(3))
        ex_(2) = sqrt(x_(3)*x_(3)-ex_(3)*ex_(3));
      else
        ex_(2) = x_(3);
      // ensure that vx > 0.1 for the feasibility of QP
      if (ex_(2)<0.1) {
        ex_(2) = 0.1;
        ex_(3) = 0.0;
        ex_(4) = 0.0;
      }

      // omega = 1/(lr+lf)*vx*tan(delta)
      // -> delta
      ex_(5) = atan(ex_(4)*(lr+lf)/ex_(2));
      // ensure that delta is correctly estimated
      if (ex_(5)>0.41) ex_(5) = 0.41;
      if (ex_(5)<-0.41)  ex_(5) = -0.41;
    }

    // for ego: suppose that we don't have vel est from /odom
    if (who_am_i==0) {
      ex_(2) = x_(3); // vx
      ex_(5) = x_(4); // delta
      ex_(3) = lr/(lr+lf)*ex_(2)*tan(ex_(5)); // vy
      ex_(4) = 1/(lr+lf)*ex_(2)*tan(ex_(5)); // omega
      if (x_(3)*x_(3)>ex_(3)*ex_(3))
        ex_(2) = sqrt(x_(3)*x_(3)-ex_(3)*ex_(3));
    }

    ex_(6) = 0.0;
    ex_(7) = s;

    ROS_DEBUG("ODOM UPDATE   --> ey %.2f, ephi %.2f, vx: %.2f, vy: %.2f, omega: %.2f, delta: %.2f, s: %.2f", ex_(0), ex_(1), ex_(2), ex_(3), ex_(4), ex_(5), ex_(7));
}

void delay_compensation(Eigen::VectorXd & ex_, double dt) {
  state_type state1 = {
    ex_(0), ex_(1), ex_(2), ex_(3), ex_(4), ex_(5), ex_(6), ex_(7),
    prev_cmd(2), prev_cmd(3),
    get_curature(ex_(7))
  };
  if (prev_cmd(2)!=0.50) integrate_in_term_of_t(state1, dt);
  for (int i = 0; i < NX; i++) ex_(i) = state1[i];

  state_type state2 = {
    ex_(0), ex_(1), ex_(2), ex_(3), ex_(4), ex_(5), ex_(6), ex_(7),
    curr_cmd(2), curr_cmd(3),
    get_curature(ex_(7))
  };
  if (curr_cmd(2)!=0.50) integrate_in_term_of_t(state2, qp_calc_period);
  for (int i = 0; i < NX; i++) ex_(i) = state2[i];

  if (ex_(0) > 1.3) ex_(0) = 1.3;
  if (ex_(0) < -1.3) ex_(0) = -1.3;

  ROS_INFO("dt = %f", dt);
  ROS_INFO("AFTER  DC     --> ey %.2f, ephi %.2f, vx: %.2f, vy: %.2f, omega: %.2f, delta: %.2f, s: %.2f", ex_(0), ex_(1), ex_(2), ex_(3), ex_(4), ex_(5), ex_(7));
}

void transform_opp_odom_to_relative_pos(Eigen::VectorXd & x_, Eigen::VectorXd & ex_) {
    double x = x_(0);
    double y = x_(1);
    double theta = x_(2);
    int box_nb = 12;

    for (int i = 0; i < 12; i++) {
      if ( (x>=LUT[i][0]&&y>=LUT[i][1]) && (x<=LUT[i][0]+LUT[i][2]&&y<=LUT[i][1]+LUT[i][3]) ) {
        box_nb = i;
        break;
      }
    }

    double s0 = LUT[box_nb][8];
    double x0 = LUT[box_nb][4];
    double y0 = LUT[box_nb][5];
    double x1 = LUT[box_nb][6];
    double y1 = LUT[box_nb][7];
    double PI = 3.14159;
    double s, ey, ephi;

    if (box_nb==0||box_nb==4||box_nb==6||box_nb==8||box_nb==10||box_nb==12) {
      // the case for LINE-type parts
      double ax = x-x0;
      double ay = y-y0;
      double bx = x1-x0;
      double by = y1-y0;

      double ab_dot = ax*bx + ay*by;
      double ab_cross = ax*by - ay*bx;
      double b_norm = sqrt(bx*bx + by*by);

      s = s0 + ab_dot/b_norm;

      double cx = ax-(ab_dot/b_norm)*(bx/b_norm);
      double cy = ay-(ab_dot/b_norm)*(by/b_norm);
      ey = sqrt(cx*cx + cy*cy);
      if (ab_cross>0) ey = -ey;

      ephi = theta - atan2(by,bx);
    } else {
      // the case for CIRCLE-type parts
      if (box_nb==2||box_nb==7) {
        // counter-clockwise cases
        double r0x = (x0+x1)/2;
        double r0y = (y0+y1)/2;
        double theta0 = atan2(y0-r0y, x0-r0x);
        double theta1 = atan2(y-r0y, x-r0x);

        double abs_dt = abs(theta1-theta0);
        if (abs_dt>PI) abs_dt = abs(abs_dt-PI*2);
        s = s0 + abs_dt*2.5;
        ey = -(sqrt((y-r0y)*(y-r0y)+(x-r0x)*(x-r0x))-2.5);
        ephi = theta - (theta1+PI/2);
      } else {
        // clockwise cases
        double r0x, r0y;
        if (box_nb==5) {
          r0x = x0;
          r0y = y1;
        } else
        if (box_nb==11) {
          r0x = x1;
          r0y = y0;
        }
        else {
          r0x = (x0+x1)/2;
          r0y = (y0+y1)/2;
        }
        double theta0 = atan2(y0-r0y, x0-r0x);
        double theta1 = atan2(y-r0y, x-r0x);

        double abs_dt = abs(theta0-theta1);
        if (abs_dt>PI) abs_dt = abs(abs_dt-PI*2);
        s = s0 + abs_dt*2.5;
        ey = sqrt((y-r0y)*(y-r0y)+(x-r0x)*(x-r0x))-2.5;
        ephi = theta - (theta1-PI/2);
      }
    }
    if (ephi>PI) ephi = ephi - PI*2;
    if (ephi<-PI) ephi = ephi + PI*2;

    ex_(0) = ey;
    ex_(1) = 0.0; //ephi;

    // vy = omega*lr
    ex_(3) = x_(4)*lr;
    // omega
    ex_(4) = x_(4);

    // vx*vx = v*v - vy*vy
    if (x_(3)*x_(3)>ex_(3)*ex_(3))
      ex_(2) = sqrt(x_(3)*x_(3)-ex_(3)*ex_(3));
    else
      ex_(2) = x_(3);
    // ensure that vx > 0.1 for the feasibility of QP
    if (ex_(2)<0.1) {
      ex_(2) = 0.1;
      ex_(3) = 0.0;
      ex_(4) = 0.0;
    }

    // omega = 1/(lr+lf)*vx*tan(delta)
    ex_(5) = atan(ex_(4)*(lr+lf)/ex_(2));
    // ensure that delta is correctly estimated
    if (ex_(5)>0.41) ex_(5) = 0.41;
    if (ex_(5)<-0.41)  ex_(5) = -0.41;

    ex_(6) = 0.0;
    ex_(7) = s;

    ROS_DEBUG("ODOM UPDATE   --> ey %.2f, ephi %.2f, vx: %.2f, vy: %.2f, omega: %.2f, delta: %.2f, s: %.2f", ex_(0), ex_(1), ex_(2), ex_(3), ex_(4), ex_(5), ex_(7));
}

void transform_to_absolute_pos(Eigen::VectorXd & pos, Eigen::VectorXd & state) {
  double s = state[0];
  if (s>87.1) s -= 87.1;

  double ey = state[1];
  double ephi = state[2];

  int box_nb = 12;
  for (int i = 0; i < 12; i++) {
    if (s>=LUT[i][8]&&s<LUT[i+1][8]) {
      box_nb = i;
      break;
    }
  }

  double s0 = LUT[box_nb][8];
  double x0 = LUT[box_nb][4];
  double y0 = LUT[box_nb][5];
  double x1 = LUT[box_nb][6];
  double y1 = LUT[box_nb][7];
  double PI = 3.14159;

  if (box_nb==0||box_nb==4||box_nb==6||box_nb==8||box_nb==10||box_nb==12) {
    // the case for LINE-type parts
    double bx = x1-x0;
    double by = y1-y0;
    double b_norm = sqrt(bx*bx + by*by);

    double px = bx/b_norm * (s-s0) + x0;
    double py = by/b_norm * (s-s0) + y0;

    double ey_theta = atan2(by, bx) + PI/2;
    pos(0) = px + ey * cos(ey_theta);
    pos(1) = py + ey * sin(ey_theta);
    pos(2) = ephi + atan2(by,bx);
  } else {
    // the case for CIRCLE-type parts
    if (box_nb==2||box_nb==7) {
      // counter-clockwise cases
      double r0x = (x0+x1)/2;
      double r0y = (y0+y1)/2;
      double theta0 = atan2(y0-r0y, x0-r0x);
      double theta1 = (s - s0)/2.5 + theta0;

      pos(0) = (2.5-ey) * cos(theta1) + r0x;
      pos(1) = (2.5-ey) * sin(theta1) + r0y;
      pos(2) = ephi + (theta1+PI/2);;
    } else {
      // clockwise cases
      double r0x, r0y;
      if (box_nb==5) {
        r0x = x0;
        r0y = y1;
      } else
      if (box_nb==11) {
        r0x = x1;
        r0y = y0;
      }
      else {
        r0x = (x0+x1)/2;
        r0y = (y0+y1)/2;
      }
      double theta0 = atan2(y0-r0y, x0-r0x);
      double theta1 = -(s - s0)/2.5 + theta0;

      pos(0) = (2.5+ey) * cos(theta1) + r0x;
      pos(1) = (2.5+ey) * sin(theta1) + r0y;
      pos(2) = ephi + (theta1-PI/2);
    }
  }
}

void obs_odes( const state_type &x , state_type &dxdt , double t) {
  double dsdt = (x[2]*cos(x[1])-x[3]*sin(x[1])) / (1-x[0]*x[10]);

  dxdt[0] = x[2]*sin(x[1]) + x[3]*cos(x[1]);
  dxdt[1] = x[4] - x[10]*dsdt;
  dxdt[2] = x[8];
  dxdt[3] = ( lr/(lr+lf) ) * ( x[9]*x[2] + x[5]*x[8] );
  dxdt[4] = (  1/(lr+lf) ) * ( x[9]*x[2] + x[5]*x[8] );
  dxdt[5] = x[9];
  dxdt[6] = 1;
  dxdt[7] = dsdt;

  dxdt[8] = 0;
  dxdt[9] = 0;

  dxdt[10] = 0;
}
void integrate_in_term_of_t(state_type &x, double delta_t) {
  typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
  size_t steps =  integrate_adaptive( make_controlled< error_stepper_type >( 1.0e-10 , 1.0e-6 ) , // abs_err = 1.0e-10 , rel_err = 1.0e-6
                    obs_odes, x, 0.0, delta_t, 1e-6 );  //...start_time, end_time, default_dt...
};

void show_trajectory_in_rviz(Eigen::MatrixXd & points, double point_size, int red) {
  visualization_msgs::Marker points_list;
  points_list.header.frame_id = "map";
  points_list.header.stamp = ros::Time::now();
  points_list.action = visualization_msgs::Marker::ADD;
  points_list.pose.orientation.w = 1.0;

  points_list.type = visualization_msgs::Marker::POINTS;
  points_list.id = 0; // opt = point;
  points_list.ns = "point_marker";

  points_list.scale.x = points_list.scale.y = point_size;
  points_list.color.b = 1.0;    // Points are blue
  points_list.color.a = 1.0;

  if (red!=0) {
    points_list.color.b = 0.0;
    points_list.color.r = 1.0;
  } else {
    if (who_am_i==0) {
      points_list.color.b = 1.0;
      points_list.color.r = 0.0;
    }
    if (who_am_i==1) {
      points_list.color.r = 1.0;
      points_list.color.g = 0.57;
      points_list.color.b = 0.1;
    }
  }

  for(int i = 0; i<points.rows(); i ++){
    geometry_msgs::Point p;
    p.x = points(i,0);
    p.y = points(i,1);
    p.z = 0.0;
    points_list.points.push_back(p);
  }

  traj_marker_pub.publish(points_list);//publishes marker
}
