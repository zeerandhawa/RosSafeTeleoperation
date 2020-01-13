/**
 * @file safe_teleop.cpp
 * @brief Safe teleoperation library implementation
 * Created by rakesh on 28/09/18.
 */
#include <limits>
#include <safe_teleop/safe_teleop.h>
#include <cmath>

namespace safe_teleop
{

SafeTeleop::SafeTeleop() :
  is_shutdown_(false),
  max_cmd_vel_age_(1.0),
  max_linear_vel_(1.0),
  max_angular_vel_(1.0),
  linear_vel_increment_(0.05),
  angular_vel_increment_(0.05),
  laser_safety_check_angle_(0.25),
  min_safety_impact_time_(0.5),
  min_safety_distance_(0.5),
  linear_vel_(0.0),
  angular_vel_(0.0),
  last_command_timestamp_(0.0),
  linear_speed_(0.2),
  angular_speed_(0.2)
{
  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  // The subscriber callback is set to the laserScanCallback method of the instantiated object of this class
  laser_scan_sub_ = global_nh.subscribe("scan", 5, &SafeTeleop::laserScanCallback, this);

  run_thread_ = boost::thread(&SafeTeleop::run, this);
  displayCurrentSpeeds();
}

SafeTeleop::~SafeTeleop()
{
  shutdown();
  // wait for the run thread to terminate
  run_thread_.join();

  geometry_msgs::Twist zero_cmd_vel;
  zero_cmd_vel.linear.x = 0;
  zero_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_cmd_vel);
}

void SafeTeleop::run()
{
  ros::Rate r(10);
  while (ros::ok() && !is_shutdown_)
  {
    auto current_timestamp = ros::Time::now().toSec();

    auto last_cmd_vel_age = current_timestamp - last_command_timestamp_;

    if (last_cmd_vel_age > max_cmd_vel_age_)
    {
      geometry_msgs::Twist zero_cmd_vel;
      zero_cmd_vel.linear.x = 0;
      zero_cmd_vel.angular.z = 0;
      cmd_vel_pub_.publish(zero_cmd_vel);
      ROS_WARN_THROTTLE(1.0, "No input for 1 sec\r");
    }
    else
    {
      auto is_safe = checkSafety(static_cast<double>(linear_vel_));
      if(is_safe)
      {
        geometry_msgs::Twist zero_cmd_vel;
        zero_cmd_vel.linear.x = linear_vel_;
        zero_cmd_vel.angular.z = angular_vel_;
        cmd_vel_pub_.publish(zero_cmd_vel);
      }

    }
    ROS_INFO_THROTTLE(1.0, "%f", (double)linear_vel_);

    r.sleep();
  }
}

void SafeTeleop::moveForward()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  linear_vel_ = double(linear_speed_);
  angular_vel_ = 0;

}

void SafeTeleop::moveBackward()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  linear_vel_ = -double(linear_speed_);
  angular_vel_ = 0;

  ROS_INFO("MOVE BACKWARD\r");
}

void SafeTeleop::rotateClockwise()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  angular_vel_ = -(double)angular_speed_;
  linear_vel_ = 0;

  ROS_INFO("ROTATE CLOCKWISE\r");
}

void SafeTeleop::rotateCounterClockwise()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  angular_vel_ = (double)angular_speed_;
  linear_vel_ = 0;


  ROS_INFO("ROTATE COUNTER CLOCKWISE\r");
}

void SafeTeleop::stop()
{
  linear_vel_ = 0;
  angular_vel_ = 0;
}


void SafeTeleop::increaseLinearSpeed()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  if(linear_speed_ <= max_linear_vel_)
  {
    linear_speed_ = linear_speed_ + linear_vel_increment_;
    ROS_INFO("Updated speed_\r");
  }
  else
  {
    ROS_WARN("Cannot increase speed beyond this limit\r");
  }
  angular_vel_ = 0;
  linear_vel_ = 0;
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseLinearSpeed()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  if(linear_speed_ > 0)
  {
    linear_speed_ = linear_speed_ - linear_vel_increment_;
    ROS_INFO("Updated speed_\r");
  }
  else
  {
    ROS_WARN("Cannot decrease speed beyond this limit\r");
  }
  angular_vel_ = 0;
  linear_vel_ = 0;
  displayCurrentSpeeds();
}

void SafeTeleop::increaseAngularSpeed()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  if(angular_speed_ <= max_angular_vel_)
  {
    angular_speed_ = angular_speed_ + angular_vel_increment_;
    ROS_INFO("Updated angular speed_\r");
  }
  else
  {
    ROS_WARN("Cannot increase speed beyond this limit\r");
  }
  linear_vel_ = 0;
  angular_vel_ = 0;
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseAngularSpeed()
{
  last_command_timestamp_ = ros::Time::now().toSec();
  if(angular_speed_ > 0)
  {
    angular_speed_ = angular_speed_ - angular_vel_increment_;
    ROS_INFO("Updated angular speed_\r");
  }
  else
  {
    ROS_WARN("Cannot decrease speed beyond this limit\r");
  }
  displayCurrentSpeeds();
  linear_vel_ = 0;
  angular_vel_ = 0;
}

bool SafeTeleop::checkSafety(double linear_vel)
{
  int flag = 1;
  auto laser_scan = getLaserScan();

  float laser_angle_max = laser_scan.angle_max;
  float laser_angle_min = laser_scan.angle_min;
  float laser_angle_inc = laser_scan.angle_increment;
  float len = (laser_angle_max - laser_angle_min)/laser_angle_inc;

  double angle_range = M_PI/12.0;
  double forward_angle_min = 0 - angle_range;
  double forward_angle_max = angle_range;

  double backward_angle_min = laser_angle_max - angle_range;
  double backward_angle_max = angle_range + laser_angle_min;

  int forward_angle_min_iteration = (int)(forward_angle_min+M_PI)/(2*M_PI)*(len+1);
  int forward_angle_max_iteration = (int)(forward_angle_max+M_PI)/(2*M_PI)*(len+1);
  int backward_angle_min_iteration = (int)(backward_angle_min+M_PI)/(2*M_PI)*(len+1);
  int backward_angle_max_iteration = (int)(((backward_angle_max+M_PI)/(2.0*M_PI))*(len+1.0));



  if(linear_vel>0)
  {

    for(int i =forward_angle_min_iteration; i <= forward_angle_max_iteration; i++)
    {

      if(laser_scan.ranges[i]<0.5)
        {
          ROS_WARN("Possible collision\r");
          return 0;
        }
    }

  }

  if(linear_vel<0)
  {
    ROS_INFO_THROTTLE(1.0, "%d\r", backward_angle_min_iteration);
    ROS_INFO_THROTTLE(1.0, "%d\r", backward_angle_max_iteration);
    for(int i =0; i <= backward_angle_max_iteration; i++)
    {
      if(laser_scan.ranges[i]<0.5)
        {
          ROS_WARN("Possible collision\r");
          return 0;
        }
    }
    for(int i = backward_angle_min_iteration; i < 128; i++)
    {
      if(laser_scan.ranges[i]<0.5)
        {
          ROS_WARN("Possible collision\r");
          return 0;
        }
    }
  }

  ROS_INFO_THROTTLE(1.0, "%f, %f, %f", laser_scan.angle_min, laser_scan.angle_max, laser_scan.angle_increment);

 return 1;
}

} // namespace safe_teleop_node
