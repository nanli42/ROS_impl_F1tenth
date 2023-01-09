#include "f110_pf_cpp_node.hpp"

bool calibration = false;

RayMarchingGPU* rmgpu;
extern ParticleFilter* pf;

extern std::ofstream file_;
extern std::ofstream file_trace_;
extern std::chrono::time_point<std::chrono::high_resolution_clock> chrono_init_time;

ParticleFilter::ParticleFilter(nav_msgs::OccupancyGrid loaded_map) {
  double MAX_RANGE_METERS = 30;
  int MAX_RANGE_PX = int( MAX_RANGE_METERS / loaded_map.info.resolution);

  this->numParticles_ = PARTICLE_NUM;
  this->init_pose_.push_back(INIT_POSE_X);
  this->init_pose_.push_back(INIT_POSE_Y);
  this->init_pose_.push_back(INIT_POSE_YAW);
  this->motion_disp_x = MOTION_DISPERSION_X;
  this->motion_disp_y = MOTION_DISPERSION_Y;
  this->motion_disp_theta = MOTION_DISPERSION_THETA;
  this->max_range_px = 30 / loaded_map.info.resolution;

  std::random_device rd{};
  std::mt19937 generator{rd()};
  std::normal_distribution<double> distribution1(0.0,0.5);
  std::normal_distribution<double> distribution2(0.0,0.4);

  for (int i = 0; i < this->numParticles_; i++) {
    particle_state particle_tmp_;

    particle_tmp_.x = init_pose_[0] + distribution1(generator);
    particle_tmp_.y = init_pose_[1] + distribution1(generator);
    particle_tmp_.theta = init_pose_[2] + distribution2(generator);
    particle_tmp_.weight = 1.0/this->numParticles_;

    this->particles_.push_back(particle_tmp_);
  }

  // ref: range_libc/pywrapper/RangeLibc.pyx, line 146 USE_ROS_MAP
  OMap map = OMap(loaded_map.info.height, loaded_map.info.width);
  for (int i = 0; i < loaded_map.info.height; i++) {
    for (int j = 0; j < loaded_map.info.width; j++) {
      if (loaded_map.data[i*loaded_map.info.width+j] == 0)
        map.grid[i][j] = false; // free space
      else
        map.grid[i][j] = true; // occupied
    }
  }
  geometry_msgs::Quaternion q = loaded_map.info.origin.orientation;
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double angle = -1.0*std::atan2(siny_cosp, cosy_cosp);

  map.world_scale = loaded_map.info.resolution;
  map.world_angle = angle;
  map.world_origin_x = loaded_map.info.origin.position.x;
  map.world_origin_y = loaded_map.info.origin.position.y;
  map.world_sin_angle = sin(angle);
  map.world_cos_angle = cos(angle);

  rmgpu = new RayMarchingGPU(map, MAX_RANGE_PX);
}

//void ParticleFilter::MCL(const std_msgs::Bool::ConstPtr& msg) {
void ParticleFilter::MCL() {
  //============================================================================//
    auto start_time = std::chrono::high_resolution_clock::now();
  //============================================================================//

  this->currentLidarMsg = this->newestLidarMsg;
  this->currentDriveMsg = this->newestDriveMsg;

  this->resample();
  double v = this->currentDriveMsg.drive.speed;
  double delta = this->currentDriveMsg.drive.steering_angle;
  this->motion_model(v, 0, v/0.3302*tan(delta));
  this->sensor_model();

  this->pub_odom();

//============================================================================//
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span_end = std::chrono::duration_cast<std::chrono::duration<double>>(end_time-chrono_init_time);
  file_ << time_span_end.count() << std::endl;
//============================================================================//
  this->particles_vis();
  this->expected_pose_vis();

  auto t_start_millis = std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count();
  double t_start_ms = (t_start_millis % 1000000) / 1000.0;
  ROS_INFO("--- start of pf ...: %7.3f [ms]", t_start_ms);
  auto t_end_millis = std::chrono::duration_cast<std::chrono::microseconds>(end_time.time_since_epoch()).count();
  double t_end_ms = (t_end_millis % 1000000) / 1000.0;
  ROS_INFO("--- end of pf ...: %7.3f [ms]\n", t_end_ms);

  file_trace_ << start_time.time_since_epoch().count() << ", " << end_time.time_since_epoch().count() << std::endl;
}

void ParticleFilter::precompute_sensor_model() {
  double z_short = 0.01;
  double z_max   = 0.07;
  double z_rand  = 0.12;
  double z_hit   = 0.75;
  double sigma_hit = 8.0;
  double pi = 3.1415926;

  int table_width = this->max_range_px + 1;
  double* table = new double[table_width*table_width];

  for (int d = 0; d < table_width; d++) {
    double norm = 0.0;
    double sum_unkown = 0.0;
    for (int r = 0; r < table_width; r++) {
      double prob = 0.0;
      float z = (float)(r-d);
      prob += z_hit * exp(-(z*z)/(2.0*sigma_hit*sigma_hit)) / (sigma_hit * sqrt(pi));
      if (r < d) prob += 2.0 * z_short * (d - r) / (float)(d);
      if (r == this->max_range_px) prob += z_max;
      norm += prob;
      table[r*table_width + d] = prob;
    }
    for (int r = 0; r < table_width; r++)
      table[r*table_width + d] /= norm;
  }
  this->table_width = table_width;
  rmgpu->set_sensor_model(table, table_width);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/*
void init_pf(nav_msgs::OccupancyGrid loaded_map) {
  pf = new ParticleFilter(loaded_map);
  pf->precompute_sensor_model();
}

void exec_pf(sensor_msgs::LaserScan currentLidarMsg) {
  pf->MCL(currentLidarMsg);
}
*/
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void ParticleFilter::motion_model(double vx, double vy, double w) {
  std::random_device rd{};
  std::mt19937 generator{rd()};
  std::normal_distribution<double> distribution1(0.0, motion_disp_x);
  std::normal_distribution<double> distribution2(0.0, motion_disp_y);
  std::normal_distribution<double> distribution3(0.0, motion_disp_theta);

  double dt = 0.02;

  for (int i = 0; i < this->numParticles_; i++) {
    double x_ = this->particles_[i].x;
    double y_ = this->particles_[i].y;
    double theta_ = this->particles_[i].theta;

    double cosine = cos(theta_);
    double sine = sin(theta_);

    this->particles_[i].x += vx*dt*cosine + vy*dt*sine + distribution1(generator);
    this->particles_[i].y += vx*dt*sine - vy*dt*cosine + distribution2(generator);
    this->particles_[i].theta += w*dt + distribution3(generator);
  }
}

void ParticleFilter::sensor_model() {
  double t1 = ros::Time::now().toSec();

  float *samples = new float[this->numParticles_*3];
  for (int i = 0; i < this->numParticles_; i++) {
    samples[i*3+0] = (float)this->particles_[i].x;
    samples[i*3+1] = (float)this->particles_[i].y;
    samples[i*3+2] = (float)this->particles_[i].theta;
  }

  std::vector<float> angles_;
  std::vector<float> obs_;
  int num_ = 0;
  for (int i = 0; i < (this->currentLidarMsg.ranges).size(); i = i + 6) {
    if (this->currentLidarMsg.ranges[i]!=0) {
      num_++;
      angles_.push_back((float)(this->currentLidarMsg.angle_min + i * this->currentLidarMsg.angle_increment));
      obs_.push_back((float)(this->currentLidarMsg.ranges[i]));
    }
  }
  float *angles = new float[num_];
  float *obs = new float[num_];
  for (int i = 0; i < num_; i++) {
    angles[i] = (float)angles_[i];
    obs[i] = (float)obs_[i];
  }

  double * weights = new double[this->numParticles_];
  int num_particles = this->numParticles_;
  int num_angles = num_;
  float *outs = new float[num_angles*num_particles];

  rmgpu->numpy_calc_range_angles(samples, angles, outs, num_particles, num_angles);

  ROS_INFO(" === numpy_calc_range_angles calc time: %f", ros::Time::now().toSec() - t1);
  double t2 = ros::Time::now().toSec();

  rmgpu->eval_sensor_model(obs, outs, weights, num_angles, num_particles);

  ROS_INFO(" === eval_sensor_model calc time: %f", ros::Time::now().toSec() - t2);

  double squash_factor = 1.0 / 2.2;
  double weight_sum = 0.0;
  for (int i = 0; i < this->numParticles_; i++) {
    weights[i] = pow(weights[i], squash_factor);
    weight_sum += weights[i];
  }
  expected_pose.x = 0;
  expected_pose.y = 0;
  expected_pose.theta = 0;
  for (int i = 0; i < this->numParticles_; i++) {
    particles_[i].weight = weights[i] / weight_sum;
    expected_pose.x += particles_[i].weight * particles_[i].x;
    expected_pose.y += particles_[i].weight * particles_[i].y;
    expected_pose.theta += particles_[i].weight * particles_[i].theta;
  }
  double laser_to_base_link = 0.275 - 0.3302/2;
  expected_pose.x -= laser_to_base_link*cos(expected_pose.theta);
  expected_pose.y -= laser_to_base_link*sin(expected_pose.theta);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
geometry_msgs::Pose pose_msg_maker(particle_state ps) {
  geometry_msgs::Pose pose_ros;
  pose_ros.position.x = ps.x;
  pose_ros.position.y = ps.y;
  pose_ros.position.z = 0.0;

  pose_ros.orientation.w = cos(ps.theta * 0.5);
  pose_ros.orientation.x = 0.0;
  pose_ros.orientation.y = 0.0;
  pose_ros.orientation.z = sin(ps.theta * 0.5);

  return pose_ros;
}

void ParticleFilter::pub_odom() {
  nav_msgs::Odometry odom;

  odom.header.stamp = this->currentLidarMsg.header.stamp;
  odom.header.frame_id = "map";

  geometry_msgs::Quaternion q;
  q.w = cos(this->expected_pose.theta * 0.5);
  q.z = sin(this->expected_pose.theta * 0.5);

  odom.pose.pose.position.x = this->expected_pose.x;
  odom.pose.pose.position.y = this->expected_pose.y;
  odom.pose.pose.orientation = q;

  pf->publish_odom_.publish(odom);
/*
  std_msgs::Bool msg;
  msg.data = true;
  pf->publish_pf_complete_.publish(msg);
*/
}

void ParticleFilter::particles_vis() {
  geometry_msgs::PoseArray particles_ros_;

  particles_ros_.header.stamp = ros::Time::now();
	particles_ros_.header.frame_id = "map";

	particles_ros_.poses.resize(numParticles_);
  for(int i = 0; i < numParticles_; i++)
	{
    particles_ros_.poses[i] = pose_msg_maker(particles_[i]);
  }
  this->publish_particlecloud_.publish(particles_ros_);
}

void ParticleFilter::expected_pose_vis() {
  geometry_msgs::PoseStamped expected_pose_;

  expected_pose_.header.stamp = ros::Time::now();
	expected_pose_.header.frame_id = "map";

  geometry_msgs::Quaternion q;
  q.w = cos(this->expected_pose.theta * 0.5);
  q.z = sin(this->expected_pose.theta * 0.5);

  expected_pose_.pose.position.x = this->expected_pose.x;
  expected_pose_.pose.position.y = this->expected_pose.y;
  expected_pose_.pose.orientation = q;
  this->publish_expected_pose_.publish(expected_pose_);
}

// ref: https://stackoverflow.com/questions/42926209/equivalent-function-to-numpy-random-choice-in-c
void ParticleFilter::resample() {
  std::vector<int> samples;
  std::vector<double> proba;
  for (int i = 0; i < this->numParticles_; i++) {
    samples.push_back(i);
    proba.push_back(this->particles_[i].weight);
  }

  int outputSize = this->numParticles_;
  std::discrete_distribution<int> distribution(proba.begin(), proba.end());

  std::random_device rd{};
  std::mt19937 generator{rd()};

  std::vector<decltype(distribution)::result_type> indices;
    indices.reserve(outputSize); // reserve to prevent reallocation
    // use a generator lambda to draw random indices based on distribution
    std::generate_n(back_inserter(indices), outputSize,
        [distribution = std::move(distribution), // could also capture by reference (&) or construct in the capture list
         generator
        ]() mutable { // mutable required for generator
            return distribution(generator);
        });

  std::vector<particle_state> new_particles_;
  for(auto const idx : indices) {
    //cout << idx << ", ";
    new_particles_.push_back(particles_[idx]);
  }
  this->particles_ = new_particles_;
}
