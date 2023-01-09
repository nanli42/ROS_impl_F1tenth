#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <chrono>

class PeriodicCtrl {
  public:
    PeriodicCtrl():
      node_handler_(ros::NodeHandle()),
      ego_drive_pub_(node_handler_.advertise<ackermann_msgs::AckermannDriveStamped>("ego_id/drive", 100)),
      opp_drive_pub_(node_handler_.advertise<ackermann_msgs::AckermannDriveStamped>("opp_id/drive", 100)),
      ego_nmpc_sub_(node_handler_.subscribe("ego_drive_nmpc", 100, &PeriodicCtrl::egoDriveCallback_NMPC, this)),
      opp_nmpc_sub_(node_handler_.subscribe("opp_drive_nmpc", 100, &PeriodicCtrl::oppDriveCallback_NMPC, this)),
      //ego_lidar_tirgger_(node_handler_.subscribe("filtered_lidar", 1, &PeriodicCtrl::egoLidarTriggerCallback, this)),
      ego_timer_(node_handler_.createTimer(ros::Duration(0.01), &PeriodicCtrl::timerCallback1, this)),
      opp_timer_(node_handler_.createTimer(ros::Duration(0.01), &PeriodicCtrl::timerCallback2, this))
    {
      ego_drive_msg_pre_.drive.steering_angle = 0.0;
      ego_drive_msg_pre_.drive.speed = 0.0;
      ego_drive_msg_.drive.steering_angle = 0.0;
      ego_drive_msg_.drive.speed = 0.0;
      opp_drive_msg_.drive.steering_angle = 0.0;
      opp_drive_msg_.drive.speed = 0.0;
    }

    void egoDriveCallback_NMPC(const ackermann_msgs::AckermannDriveStamped &drive_msg) {
      //============================================================================//
        auto start_time = std::chrono::high_resolution_clock::now();
      //============================================================================//

      auto millis = std::chrono::duration_cast<std::chrono::microseconds>(start_time.time_since_epoch()).count();
      double t_ms = (millis % 1000000) / 1000.0;
      ROS_INFO("--- in periodic ctrler - egoDriveCallback_NMPC ...: %7.3f [ms]", t_ms);
/*
      if (!isnan(drive_msg.drive.speed) && !isnan(drive_msg.drive.steering_angle)) ego_drive_msg_pre_ = drive_msg;
      else {
        ego_drive_msg_pre_.drive.steering_angle = 0.0;
        ego_drive_msg_pre_.drive.speed = 0.0;
      }
*/
      if (!isnan(drive_msg.drive.speed) && !isnan(drive_msg.drive.steering_angle)) ego_drive_msg_ = drive_msg;
      else {
        ego_drive_msg_.drive.steering_angle = 0.0;
        ego_drive_msg_.drive.speed = 0.0;
      }
    }

    void oppDriveCallback_NMPC(const ackermann_msgs::AckermannDriveStamped &drive_msg) {
      if (!isnan(drive_msg.drive.speed) && !isnan(drive_msg.drive.steering_angle)) opp_drive_msg_ = drive_msg;
      else {
        opp_drive_msg_.drive.steering_angle = 0.0;
        opp_drive_msg_.drive.speed = 0.0;
      }
      if (opp_drive_msg_.drive.speed > 1.5)
        opp_drive_msg_.drive.speed = 1.5;
    }

    void egoLidarTriggerCallback(const sensor_msgs::LaserScan& msg) {
      //ego_drive_msg_ = ego_drive_msg_pre_;
    }

    void timerCallback1(const ros::TimerEvent& event) {
      //ROS_DEBUG("<- 1 -- TIME NOW", event.current_real);
      ego_drive_pub_.publish(ego_drive_msg_);
    }
    void timerCallback2(const ros::TimerEvent& event) {
      //ROS_DEBUG("<- 2 == TIME NOW", event.current_real);
      opp_drive_pub_.publish(opp_drive_msg_);
    }

  private:
    ros::NodeHandle node_handler_;

    ros::Publisher ego_drive_pub_;
    ros::Publisher opp_drive_pub_;

    ros::Subscriber ego_nmpc_sub_;
    ros::Subscriber opp_nmpc_sub_;

    ros::Timer ego_timer_;
    ros::Subscriber ego_lidar_tirgger_;
    ros::Timer opp_timer_;

    ackermann_msgs::AckermannDriveStamped ego_drive_msg_pre_;
    ackermann_msgs::AckermannDriveStamped ego_drive_msg_;
    ackermann_msgs::AckermannDriveStamped opp_drive_msg_;

};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "periodic_ctrl_node");

    PeriodicCtrl periodic_ctrler;
    ros::spin();

    return 0;
}
