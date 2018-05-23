/**
 *      @file  r_automatic.cpp
 *      @brief
 *
 *
 *      @author   Luís Sarmento - sarmento@ua.pt
 *
 *    @internal
 *      Created  2-May-2018
 *      Company  University of Aveiro
 *
 * =====================================================================================
 */

//=======================================================================================
//================================ LIBS / DEFINES
//=======================================
//=======================================================================================

#include "geometry_msgs/Twist.h"
#include "r_platform/navi.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <vector>

//=======================================================================================
//===================================== CLASS
//===========================================
//=======================================================================================

bool robot_allowed = true;

class TeleopRobonuc
{
public:
  TeleopRobonuc();

  const r_platform::navi &vel_msg() const;

  void publish_vel_msg();

  void modeDecider(void);

private:
  void autoNav(const geometry_msgs::Twist::ConstPtr &vel);

  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  void laserDanger(const sensor_msgs::LaserScan::ConstPtr &msg);

  int rad2LaserCoord(float rad, int laser_size);

  ros::NodeHandle nh_;

  int linear_,
      angular_;             // id of angular and linear axis (position in the array)
  float l_scale_, a_scale_; // linear and angular scale
  ros::Publisher vel_pub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber laser_sub_;

  r_platform::navi vel_msg_;

  float l_vel_, a_vel_;

  float auto_l_vel_ = 0, auto_a_vel_ = 0;

  float joy_l_vel_, joy_a_vel_;

  bool laser_danger_;

  int mode_; // mode==1 -> manual; mode==2->automatic
};

TeleopRobonuc::TeleopRobonuc()
    : linear_(1), angular_(3), l_scale_(0.025), a_scale_(0.025)
{

  vel_pub_ = nh_.advertise<r_platform::navi>("navi_commands", 20);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 20,
                                             &TeleopRobonuc::joyCallback, this);

  vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 20,
                                                 &TeleopRobonuc::autoNav, this);

  laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
      "scan", 20, &TeleopRobonuc::laserDanger, this);
}

const r_platform::navi &TeleopRobonuc::vel_msg() const { return vel_msg_; }

void TeleopRobonuc::publish_vel_msg() { vel_pub_.publish(vel_msg()); }

//=======================================================================================
//=================================== CALLBACKS
//=========================================
//=======================================================================================

// tentar definir um rate de publicação fixo, e quando entra no loop é que vai
// fazer subscribe.. Assim garantiam-se os 20 Hz

int TeleopRobonuc::rad2LaserCoord(float rad, int laser_size)
{
  int l_coord = round((0.5 + rad / (3 * M_PI)) * laser_size);
  return l_coord;
}

void TeleopRobonuc::autoNav(const geometry_msgs::Twist::ConstPtr &vel)
{

  auto_l_vel_ = vel->linear.x;
  auto_a_vel_ = vel->angular.z;
}

void TeleopRobonuc::laserDanger(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  std::vector<float> laser;
  laser = msg->ranges;
  int size_laser = laser.size();
  float l_vel = joy_l_vel_;
  float a_vel = joy_a_vel_;
  float alpha = atan(a_vel / l_vel);
  int n_danger = 0;

  if (l_vel >= 0)
  {
    if (l_vel > 0 || a_vel != 0)
    {
      // test zone 1
      for (int i =
               TeleopRobonuc::rad2LaserCoord(alpha - 2 * M_PI / 3, size_laser);
           i < TeleopRobonuc::rad2LaserCoord(alpha + 2 * M_PI / 3, size_laser);
           i++)
      {
        ROS_INFO("i=%d", i);
        if (laser[i] < l_vel + 0.1 && laser[i] > 0.1)
        {
          n_danger++;
          ROS_INFO("Zona 1 - Danger");
        }
      }

      // test zone 2
      for (int i = TeleopRobonuc::rad2LaserCoord(-M_PI / 2, size_laser);
           i < TeleopRobonuc::rad2LaserCoord(M_PI / 2, size_laser); i++)
      {
        if (laser[i] < 0.37 && laser[i] > 0.1)
        {
          n_danger++;
          ROS_INFO("Zona 2 - Danger");
        }
      }

      // Test zone 3
      // if (alpha < 0)
      // {
      //   for (int i = TeleopRobonuc::rad2LaserCoord(11 * M_PI / 18,
      //   size_laser);
      //        i < TeleopRobonuc::rad2LaserCoord(3 * M_PI / 4, size_laser);
      //        i++){
      //          if (laser[i]<37+4*a_vel/())
      //        }
      // }
    }

    if (n_danger > 5)
    {
      laser_danger_ = true;
      ROS_INFO("Zona 1 - Danger");
    } // obstáculo na direção do movimento
    else
    {
      laser_danger_ = false;
    }
  }
}

void TeleopRobonuc::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{

  r_platform::navi vel_msg;

  // decrease velocity rate
  if (joy->buttons[4] == 1 && a_scale_ > 0 && l_scale_ > 0.0)
  {
    l_scale_ = l_scale_ - 0.025;
    a_scale_ = a_scale_ - 0.025;

    ROS_INFO("DEC v_rate l[%f] a[%f]", l_scale_, a_scale_);
  }

  // increase velocity rate
  if (joy->buttons[5] == 1 && a_scale_ < 0.5 && l_scale_ < 0.5)
  {
    l_scale_ = l_scale_ + 0.025;
    a_scale_ = a_scale_ + 0.025;

    ROS_INFO("INC v_rate l[%f] a[%f]", l_scale_, a_scale_);
  }

  // deadman switch
  // if (joy->buttons[6] == 1)
  if (joy->axes[2] == -1 & joy->axes[5] == -1)
  {
    joy_l_vel_ = l_scale_ * joy->axes[linear_];
    joy_a_vel_ = a_scale_ * joy->axes[angular_];
    mode_ = 1;

    // vel_msg_.robot = 0;

    if (joy->buttons[0] == 0 && joy->buttons[1] == 0 && joy->buttons[2] == 0 &&
        joy->buttons[3] == 0 && joy->buttons[7] == 0)
    {
      robot_allowed = true;
    }

    // Robot Action
    if (joy->buttons[0] == 1 && robot_allowed)
    {
      vel_msg_.robot = 1;
      robot_allowed = false;
    }
    else if (joy->buttons[1] == 1 && robot_allowed)
    {
      vel_msg_.robot = 2;
      robot_allowed = false;
    }
    else if (joy->buttons[2] == 1 && robot_allowed)
    {
      vel_msg_.robot = 3;
      robot_allowed = false;
    }
    else if (joy->buttons[3] == 1 && robot_allowed)
    {
      vel_msg_.robot = 4;
      robot_allowed = false;
    }
    else if (joy->buttons[7] == 1 && robot_allowed)
    {
      vel_msg_.robot = 5;
      robot_allowed = false;
    }
  }
  // else if (joy->axes[2] == -1 & joy->axes[5] == -1 & vel_msg_.robot != 7)
  // {
  //   {
  //     vel_msg_.linear_vel = l_vel_;
  //     vel_msg_.angular_vel = a_vel_;
  //     // vel_msg_.robot = 0;
  //   }
  // }
  else if (joy->axes[2] == -1 & joy->axes[5] == 1)
  { // Automatic navigation
    mode_ = 2;
  }
  else
  {
    vel_msg_.linear_vel = 0;
    vel_msg_.angular_vel = 0;
    vel_msg_.robot = 0;
    mode_ = 0;
  }

  // vel_pub_.publish(vel_msg);
}

void TeleopRobonuc::modeDecider(void)
{
  if (mode_ == 2) // automatic
  {
    vel_msg_.linear_vel = auto_l_vel_;
    vel_msg_.angular_vel = auto_a_vel_;
  }

  else if (mode_ == 1) // manual
  {
    if (!laser_danger_)
    {
      vel_msg_.linear_vel = joy_l_vel_;
      vel_msg_.angular_vel = joy_a_vel_;
    }
    else
    {
      vel_msg_.linear_vel = auto_l_vel_;
      vel_msg_.angular_vel = auto_a_vel_;
    }
  }

  else
  {
    vel_msg_.linear_vel = 0;
    vel_msg_.angular_vel = 0;
  }
}
//=======================================================================================
//====================================== MAIN
//===========================================
//=======================================================================================

int main(int argc, char **argv)
{

  ros::init(argc, argv, "teleop_robonuc");

  TeleopRobonuc teleop_robonuc;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {

    teleop_robonuc.modeDecider();

    teleop_robonuc.publish_vel_msg();

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
