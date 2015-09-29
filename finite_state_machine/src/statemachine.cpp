#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
//#include <tf/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <sensor_msgs/Joy.h>
#include <string>
#define PI 3.14159265

enum state_t {start, localize, move_mine, move_bin, mine, approach_bin, 
              raise_bin_neutral, lower_bin_neutral, lower_bin_mine,
              dump, manual};

class SubscribeAndPublish
{
public:
  SubscribeAndPublish() : 
    miningXpoints {2.92,2.63,2.4,1.89,1.38,1.15,0.86,0.7},  
    miningYpoints {5.9,6.25,6.45,6.6,6.55,6.25,5.9,5.2}, 
    miningYawpoints {120.*PI/180.,144.*PI/180.,160.*PI/180.,PI,
                     200.*PI/180.,216.*PI/180.,240.*PI/180.,-PI/2.}
                     /*The x and y appear to be coordinate pairs for places on the field that are good 
                     potential cantidates for mining. The yaw points are the angle that the robot should be facing
                     when mining them. They are coded in radians*/
  {
    
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    //Topic you want to subscribe
    sub_amcl = n_.subscribe("/amcl_pose", 1, &SubscribeAndPublish::callback_amcl, this);
    sub_cmd = n_.subscribe("/state_machine_trigger", 1,
                           &SubscribeAndPublish::callback_cmd, this);
    sub_joy = n_.subscribe("/joy", 1, &SubscribeAndPublish::callback_joy, this);

    // Parameters
    ros::param::set("/bin/position", 0);
    ros::param::set("/bin/command", 0);
    ros::param::set("/bin/turbine", 0);

    // Goals
    goalstate = move_mine;
  curstate = start;
  ROS_INFO("State: Start");
//curstate = manual;
//ROS_INFO("State: Manual");
    curgoal = geometry_msgs::Pose();
    // Mining area and bin locations
    mineLocation = geometry_msgs::Pose();
    mineLocation.position.x = 3.08;
    mineLocation.position.y = 5.2;
    mineLocation.orientation = tf::createQuaternionMsgFromYaw(PI/2.);
    farBinLocation = geometry_msgs::Pose();
    farBinLocation.position.x = 1.89;
    farBinLocation.position.y = 1.65;
    farBinLocation.orientation = tf::createQuaternionMsgFromYaw(-PI/2.);
    nearBinLocation = geometry_msgs::Pose();
    nearBinLocation.position.x = 1.89;
    nearBinLocation.position.y = 0.65;
    nearBinLocation.orientation = tf::createQuaternionMsgFromYaw(-PI/2.);
    numminegoals = 8;

    frame_id = "/map";
    // Loop until ros::Time::Now() != 0
    while (ros::Time::now().sec == 0) {}
  };

  void callback_amcl(const geometry_msgs::PoseWithCovarianceStamped & curloc1)
  {
    curloc = curloc1;
    curloc.pose.pose.position.y = curloc1.pose.pose.position.x;
    curloc.pose.pose.position.x = 3.78 - curloc1.pose.pose.position.y;
    curloc.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(curloc1.pose.pose.orientation) + PI/2.);
  };

  void callback_cmd(const std_msgs::Bool msg)
  {
    ros::param::get("/bin/position", bin_position);

    switch (curstate)
    { 
      // Start State
      case start:
        curgoal = curloc.pose.pose;
        break;

      // Localize
      case localize:
        if (lost(curloc.pose))
        {
          curgoal = curloc.pose.pose;
          curgoal.orientation = tf::createQuaternionMsgFromYaw(
              tf::getYaw(curgoal.orientation) + PI);
        }
        else
        {
          curstate = goalstate;
          if (curstate == move_mine)
          {
            curgoal = mineLocation;
            ROS_INFO("State: Move to Mining Area");
          }
          else if (curstate == move_bin)
          {
            curgoal = farBinLocation;
            ROS_INFO("State: Move to Bin");
          }
        }
        break;

      // Move to Mining Area
      case move_mine:
        if (lost(curloc.pose))
        {
          curstate = localize;
          ROS_INFO("State: Localize");
        }
        else if (reachedGoal(curloc.pose.pose))
        {
          curgoal = curloc.pose.pose;
          curstate = lower_bin_mine;
          ros::param::set("/bin/command", -1);
          ROS_INFO("State: Lowering Bin for Mining");
        }
        else
        {
          curgoal = mineLocation;
        }
        break;
      
      // Lower Bin for Mining
      case lower_bin_mine:
        if (bin_position == -1)
        {
          curstate = mine;
          minecount = 0;
          curgoal.position.x = miningXpoints[minecount];
          curgoal.position.y = miningYpoints[minecount];
          curgoal.orientation = tf::createQuaternionMsgFromYaw(miningYawpoints[minecount]);
          ros::param::set("/bin/turbine", true);
          ROS_INFO("State: Mining");
        }
        else
        {
          curgoal = curloc.pose.pose;
        }
        break;

      case mine:
        if (reachedGoal(curloc.pose.pose))
        {
          minecount++;
          ROS_INFO("mincount = %d", minecount);
          if (minecount == numminegoals)
          {
            curgoal = curloc.pose.pose;
            curstate = raise_bin_neutral;
            ros::param::set("/bin/command", 0);
            ros::param::set("/turbine/command", false);
            ROS_INFO("State: Raise bin to Neutral Position");
            break;
          }
        }
        curgoal.position.x = miningXpoints[minecount];
        curgoal.position.y = miningYpoints[minecount];
        curgoal.orientation = tf::createQuaternionMsgFromYaw(miningYawpoints[minecount]);
        break;

      case raise_bin_neutral:
        if (bin_position == 0)
        {
          curgoal = farBinLocation;
          goalstate = move_bin;
          curstate = move_bin;
          ROS_INFO("State: Move to Bin");
        }
        else
        {
          curgoal = curloc.pose.pose;
        }
        break;

      // Move to Bin
      case move_bin:
        if (lost(curloc.pose))
        {
          curstate = localize;
          ROS_INFO("State: Localize");
        }
        else if (reachedGoal(curloc.pose.pose))
        {
          curstate = approach_bin;
          ROS_INFO("State: Approach Bin");
          curgoal = nearBinLocation;
        }
        else
        {
          curgoal = farBinLocation;
        }
        break;
      
      case approach_bin:
        if (reachedGoal(curloc.pose.pose))
        {
          curstate = dump;
          curgoal = curloc.pose.pose;
          ros::param::set("/bin/command", 1);
          ROS_INFO("State: Dumping");
        }
        else
        {
          curgoal = nearBinLocation;
        }
        break;

      case dump:
        if (bin_position == 1)
        {
          curstate = lower_bin_neutral;
          curgoal = curloc.pose.pose;
          ros::param::set("/bin/command", 0);
          ROS_INFO("State: Lower Bin to Neutral Position");
        }
        curgoal = curloc.pose.pose;
        break;

      case lower_bin_neutral:
        if (bin_position == 0)
        {
          curgoal = mineLocation;
          curstate = move_mine;
          goalstate = move_mine;
          ROS_INFO("State: Move to Mining Area");
        }

      case manual:
        return;
        break;

    } // end switch

    if (curstate != manual)
    {
      geometry_msgs::PoseStamped goalStamped = geometry_msgs::PoseStamped();
      goalStamped.pose = curgoal;
      goalStamped.header.stamp = ros::Time::now();
      goalStamped.header.frame_id = frame_id;
      pub_.publish(goalStamped);
    }
  };

  void callback_joy(const sensor_msgs::Joy::ConstPtr& joy)
  {
    if (curstate == start && joy->buttons[7])
    {
      curstate = localize;
      ROS_INFO("State: Localize");
    }
    if (joy->buttons[0] && curstate != manual)
    {
      curstate = manual;
      //system("rosrun topic_tools mux joy3 joy2 joy mux:=mux_joy &");
      system("rosrun topic_tools mux_select mux_joy joy &");
      system("rosnode kill move_base_node &");
      system("rosnode kill amcl &");
      ROS_INFO("State: Manual");
    }
    if (joy->axes[7] > 0.9 && curstate == manual)
    {
//    int position;
//    ros::param::get("/bin/position", position);
//    if (position < 1)
//    {
//      ros::param::set("/bin/command", position+1);
//    }
      int command;
      ros::param::get("/bin/command", command);
      if (command < 1)
      {
        ros::param::set("/bin/command", 1); //command+1);
      //ros::Duration(1.0).sleep();
      }
    }
    if (joy->axes[7] < -0.9 && curstate == manual)
    {
//    int position;
//    ros::param::get("/bin/position", position);
//    if (position > -1)
//    {
//      ros::param::set("/bin/command", position-1);
//    }
      int command;
      ros::param::get("/bin/command", command);
      if (command > -1)
      {
        ros::param::set("/bin/command", -1); //command-1);
      //ros::Duration(1.0).sleep();
      }
    }
    if (joy->buttons[3] && curstate == manual)
    {
      ros::param::set("/bin/command", 0);
    }
    if (joy->buttons[4] && curstate == manual)
    {
      ros::param::set("/bin/turbine", false);
    }
    if (joy->buttons[5] && curstate == manual)
    {
      ros::param::set("/bin/turbine", true);
    }
  };

  bool lost(const geometry_msgs::PoseWithCovariance & curlocation)
  {
    double xx = curlocation.covariance[0];
    double yy = curlocation.covariance[7];
    double psipsi = curlocation.covariance[35];

    return (xx > 0.03 || yy > 0.03 || psipsi > 0.03);
  };

  bool reachedGoal(const geometry_msgs::Pose & curlocation)
  {
    double xdist = curlocation.position.x - curgoal.position.x;
    double ydist = curlocation.position.y - curgoal.position.y;
    double sqdist = xdist * xdist + ydist * ydist;
    double thetadist = tf::getYaw(curlocation.orientation) - 
                       tf::getYaw(curgoal.orientation);
    return (sqdist < .2) && (thetadist*thetadist < PI/15.);
  };

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_amcl;
  ros::Subscriber sub_cmd;
  ros::Subscriber sub_joy;
  geometry_msgs::Pose curgoal;
  geometry_msgs::Pose mineLocation, farBinLocation, nearBinLocation;
  geometry_msgs::PoseWithCovarianceStamped curloc; 
  std::string frame_id;
  state_t curstate;
  state_t goalstate;
  unsigned int minecount;
  double miningXpoints[8]; 
  double miningYpoints[8];
  double miningYawpoints[8];
  unsigned int numminegoals;
  int bin_position;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "finite_state_machine");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
