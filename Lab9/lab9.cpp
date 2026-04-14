// NAMES: Jordi Fraile Raton, Valentina Parodi
// CLASS: EGRS372
// PROJECT: Lab 9
// PURPOSE: Modify Turtlebot parameters via both terminal and code and simulate conditions that require MR to change behavior

// INCLUDE/DEFINE NECESSARY CONSTANTS/LIBRARIES
#include "ros/ros.h"      // INCLUDE ROS (ROBOT OPERATING SYSTEM)
#include <sstream>        // LIBRARY THAT PROVIDES STRING CLASSES
#include <iostream>       // FOR INPUTS AND OUTPUTS FOR CIN (INPUT)
#include <fstream>        // PROVIDES CLASSES TO PERFORM OPERATIONS ON FILES
#include <string>         // FOR STRING DECLARATIONS
#include "geometry_msgs/PoseStamped.h"              // PUBLISHING A GEOMETRY_MSGS/...
#include "std_msgs/Byte.h"                          // FOR READING THE PUSHBUTTON / BUMPER TOPIC
#include "std_msgs/Int32.h"                         // FOR SENDING DATA TO THE 7-SEGMENT DISPLAY
#include "move_base_msgs/MoveBaseActionResult.h"    // FOR READING move_base RESULT STATUS
#include <move_base_msgs/MoveBaseAction.h>          // TO DEFINE ACTION MESSAGES FOR NAVIGATING
#include <actionlib/client/simple_action_client.h>  // TO ALLOW USERS TO SEND GOALS AND RECEIVE FEEDBACK FROM AN ActionServer
#include <ros/package.h>                            // INCLUDE ROS PACKAGE PATH
#define PI 3.14159265359                            // RATIO OF CIRCLE, CIRCUMFERENCE:DIAMETER

// DYNAMIC RECONFIGURE
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/service.h>

// COSTUM MESSAGE FOR MR BEHAVIOR
#include "lab9/behavior_params.h"

// MAIN PROGRAM: MR PERFORMS DIFFERENT BEHAVIOR WHEN MOVING BETWEEN 5 DIFFERENT POINTS BASED ON A GIVEN PARAMETERS FILE
int main(int argc, char **argv)
{
  // NAMES THE PROGRAM (FOR VISUAL PURPOSES)
  ros::init(argc, argv, "lab9");

  // NODEHANDLE::ADVERTISE() RETURNS A ROS::PUBLISHER OBJECT,
  // WHICH SERVES TWO PURPOSES: 1) IT CONTAINS A PUBLISH() METHOD THAT
  // LETS YOU PUBLISH MESSAGES ONTO THE TOPIC IT WAS CREATED WITH, AND
  // 2) WHEN IT GOES OUT OF SCOPE, IT WILL AUTOMATICALLY UNADVERTISE.
  ros::NodeHandle n;

  // SETS THE FREQUENCY FOR WHICH THE PROGRAM SLEEPS AT
  ros::Rate loop_rate(10);

  // PUBLISHER FOR COSTUM MESSAGE
  ros::Publisher param_pub = n.advertise<lab9::behavior_params>("behavior_updates", 1);

  // DECLARE DYNAMIC RECONFIGURE VARIABLES
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  // CONNECT ACTION CLIENT TO move_base_msgstion FOR NAVIGATION
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  ac.waitForServer(); // CHECK CONNECTION HAS BEEN ESTABLISHED 

  // OPEN PATH FILE AND READ parameters.txt
  std::string filename = ros::package::getPath("lab9") + "/parameters.txt";
  std::ifstream infile(filename.c_str());

  // CHECK FILE HAS BEEN OPENED
  if (!infile.is_open())
  {
    ROS_ERROR("Cannot open parameters.txt");
    return 1;
  }

  std::string line; // DECLARE STRING TO READ LINE
  int i = 0; // INITIALIZE VARIABLE TO KEEP TRACK OF ROBOT POSITION

  // LOOP THROUGH THE 5 LINES OF THE FILE
  while (getline(infile, line) && i < 5 && ros::ok())
  {
    std::stringstream ss(line); // CREATE A STRING STREAM OBJECT
    std::string s1, s2, s3, s4; // DECLARE STRINGS TO STORE EACH VALUE FROM THE LINE

    getline(ss, s1, ','); // READ FIRST VALUE (MAX LINEAR SPEED)
    getline(ss, s2, ','); // READ SECOND VALUE (ALLOW BACKWARD)
    getline(ss, s3, ','); // READ THIRD VALUE (MAX ROTATIONAL SPEED)
    getline(ss, s4, ','); // READ FOURTH VALUE (ADJUST ORIENTATION)

    double max_lin = atof(s1.c_str()); // CONVERT STRING TO DOUBLE FOR LINEAR SPEED
    double max_rot = atof(s3.c_str()); // CONVERT STRING TO DOUBLE FOR ROTATIONAL SPEED

    // INITIALIZE FLAGS
    bool allow_back = false;
    bool adjust_orient = false;

    if (s2 == "t" || s2 == "T")
    {
      allow_back = true; // ENABLE BACKWARD MOTION IF TRUE
    }
    if (s4 == "t" || s4 == "T")
    {
      adjust_orient = true; // ENABLE ORIENTATION ADJUSTMENT IF TRUE
    }
    // CLEAR PREVIOUS PARAMETERS
    conf.doubles.clear();

    // SET THE MAXIMUM FORWARD LINEAR SPEED OF THE ROBOT
    double_param.name = "max_vel_x";
    double_param.value = max_lin;
    conf.doubles.push_back(double_param); // ADD PARAMETER TO CONFIG LIST

    // SET THE MINIMUM LINEAR SPEED
    double_param.name = "min_vel_x";
    if (allow_back)
      double_param.value = -0.13; // ALLOW BACKWARD MOVEMENT
    else
      double_param.value = 0.0;
    conf.doubles.push_back(double_param); // ADD PARAMETER TO CONFIG LIST

    // SET THE MAXIMUM ROTATIONAL ANGULAR SPEED OF THE ROBOT
    double_param.name = "max_rot_vel";
    double_param.value = max_rot;
    conf.doubles.push_back(double_param); // ADD PARAMETER TO CONFIG LIST

    // SET HOW PRECISE THE ROBOT HAS TO BE WITH ITS ORIENTATION
    double_param.name = "yaw_goal_tolerance";
    if (adjust_orient)
      double_param.value = 0.17;
    else
      double_param.value = 10.0;
    conf.doubles.push_back(double_param); // ADD PARAMETER TO CONFIG LIST

    // CALL SERVICE
    srv_req.config = conf;
    ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
    ROS_INFO("Updated parameters for point %d", i+1);

    // PUBLISH COSTUM MESSAGE
    lab9::behavior_params param_msg;
    param_msg.max_linear_speed = max_lin;
    param_msg.allow_backward = allow_back;
    param_msg.max_rotational_speed = max_rot;
    param_msg.adjust_orientation = adjust_orient;
    param_pub.publish(param_msg);

    // DECLARE goal VARIABLE FOR POSITIONS
    move_base_msgs::MoveBaseGoal goal;

    // SET MAP FRAME TO BE USED 
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // SEND ROBOT TO POINT 1
    if (i == 0)
    {
      goal.target_pose.pose.position.x = 1.0;
      goal.target_pose.pose.position.y = 0.5;
    }
    // SEND ROBOT TO POINT 2
    if (i == 1)
    {
      goal.target_pose.pose.position.x = 3.0;
      goal.target_pose.pose.position.y = 0.5;
    }
    // SEND ROBOT TO POINT 3
    if (i == 2)
    {
      goal.target_pose.pose.position.x = 5.0;
      goal.target_pose.pose.position.y = 0.8;
    }
    // SEND ROBOT TO POINT 4
    if (i == 3)
    {
      goal.target_pose.pose.position.x = 7.0;
      goal.target_pose.pose.position.y = 0.5;
    }
    // SEND ROBOT TO POINT 5
    if (i == 4)
    {
      goal.target_pose.pose.position.x = 10.0;
      goal.target_pose.pose.position.y = 0.8;
    }
    // SET MR ORIENTATION TO BE CONSTANT
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal %d", i+1);

    ac.sendGoal(goal); // SEND GOAL TO move_base ACTION SERVER
    ac.waitForResult(); // WAIT FOR GOAL
    // CHECK NAVIGATION RESULT
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Reached point %d", i+1);
    }
    else
    {
      ROS_WARN("Failed point %d", i+1);
    }

    ros::spinOnce();
    loop_rate.sleep();
    i++; // INCREMENT POSITION VARIABLE WHEN ONE POSITION IS REACHED
  }
  infile.close(); // CLOSE parameters.txt FILE
  ROS_INFO("Done."); // END PROGRAM

  return 0;
}
