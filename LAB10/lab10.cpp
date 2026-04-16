// NAMES: Jordi Fraile Raton, Valentina Parodi
// CLASS: EGRS372
// PROJECT: Lab 10
// PURPOSE: Create safety zones within ROS and modify Turtlebot behavior based on safety zones such as a crosswalk and human proximity

// INCLUDE/DEFINE NECESSARY CONSTANTS/LIBRARIES
#include "ros/ros.h"      // INCLUDE ROS (ROBOT OPERATING SYSTEM)
#include <sstream>        // LIBRARY THAT PROVIDES STRING CLASSES
#include <fstream>        // PROVIDES CLASSES TO PERFORM OPERATIONS ON FILES
#include <string>         // FOR STRING DECLARATIONS
#include <cmath>          // FOR DISTANCE CALCULATION
#include <move_base_msgs/MoveBaseAction.h>          // TO DEFINE ACTION MESSAGES FOR NAVIGATING
#include <move_base_msgs/MoveBaseActionFeedback.h>  // TO GET ROBOT POSITION FROM move_base FEEDBACK
#include <actionlib/client/simple_action_client.h>  // TO ALLOW USERS TO SEND GOALS AND RECEIVE FEEDBACK FROM AN ActionServer
#include <ros/package.h>                            // INCLUDE ROS PACKAGE PATH
#include "geometry_msgs/Point.h"                    // FOR HUMAN POSITION
#include "std_msgs/Bool.h"                          // FOR motor_power TOPIC
#define PI 3.14159265359                            // RATIO OF CIRCLE, CIRCUMFERENCE:DIAMETER

// DYNAMIC RECONFIGURE
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/service.h>

// COSTUM MESSAGE FOR MR BEHAVIOR
#include "lab9/behavior_params.h"

// DEFINE CORNERS OF CROSSWALK
#define X1 3.0
#define Y1 8.0
#define X2 6.0
#define Y2 8.5

// GLOBAL VARIABLES FOR CURRENT ROBOT POSITION
double robot_x = 0.0;
double robot_y = 0.0;

// GLOBAL VARIABLES FOR CURRENT HUMAN POSITION
double human_x = 100.0;
double human_y = 100.0;

// CALLBACK FUNCTION FOR HUMAN POSITION
void human_callback(const geometry_msgs::Point::ConstPtr& msg)
{
  // SAVE CURRENT HUMAN POSITION
  human_x = msg->x;
  human_y = msg->y;
}

// CALLBACK FUNCTION FOR ROBOT POSITION
void feedback_callback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
  // SAVE CURRENT ROBOT FROM move_base FEEDBACK
  robot_x = msg->feedback.base_position.pose.position.x;
  robot_y = msg->feedback.base_position.pose.position.y;
}

// MAIN PROGRAM: MR PERFORMS DIFFERENT BEHAVIOR WHEN MOVING BETWEEN 5 DIFFERENT POINTS BASED ON A GIVEN PARAMETERS FILE AND CHANGES ITS MOTION BASED ON SAFETY ZONES
int main(int argc, char **argv)
{
  // NAMES THE PROGRAM (FOR VISUAL PURPOSES)
  ros::init(argc, argv, "lab10");

  // NODEHANDLE::ADVERTISE() RETURNS A ROS::PUBLISHER OBJECT,
  // WHICH SERVES TWO PURPOSES: 1) IT CONTAINS A PUBLISH() METHOD THAT
  // LETS YOU PUBLISH MESSAGES ONTO THE TOPIC IT WAS CREATED WITH, AND
  // 2) WHEN IT GOES OUT OF SCOPE, IT WILL AUTOMATICALLY UNADVERTISE.
  ros::NodeHandle n;

  // SETS THE FREQUENCY FOR WHICH THE PROGRAM SLEEPS AT
  ros::Rate loop_rate(10);

  // PUBLISHER FOR COSTUM MESSAGE
  ros::Publisher param_pub = n.advertise<lab9::behavior_params>("behavior_updates", 1);

  // PUBLISHER FOR MOTOR POWER
  ros::Publisher motor_pub = n.advertise<std_msgs::Bool>("/motor_power", 1);

  // SUBSCRIBER TO HUMAN POSITION
  ros::Subscriber human_sub = n.subscribe("human", 1, human_callback);

  // SUBSCRIBER TO ROBOT POSITION FROM move_base FEEDBACK
  ros::Subscriber feedback_sub = n.subscribe("/move_base/feedback", 1, feedback_callback);

  // DECLARE DYNAMIC RECONFIGURE VARIABLES
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  // CONNECT ACTION CLIENT TO move_base ACTION FOR NAVIGATION
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

    // WHILE LOOP FOR WHEN THE ROBOT IS MOVING TO THE CURRENT GOAL
    while (ros::ok() && ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ros::spinOnce(); // UPDATE SUBSCRIBER CALLBACKS

      // STORE CURRENT SPEEDS BASED ON THE FILE VALUES
      double current_max_lin = max_lin;
      double current_max_rot = max_rot;

      // CHECK IF THE ROBOT IS INSIDE THE CROSSWALK SHAPE
      if (robot_x >= X1 && robot_x <= X2 && robot_y >= Y1 && robot_y <= Y2)
      {
        // IF ROBOT ENTERS CROSSWALK, REDUCE LINEAR SPEED ONLY IF IT IS FASTER THAN 0.13
        if (current_max_lin > 0.13)
        {
          current_max_lin = 0.13;
        }

        // IF ROBOT ENTERS CROSSWALK, REDUCE ANGULAR SPEED ONLY IF IT IS FASTER THAN 1.0
        if (current_max_rot > 1.0)
        {
          current_max_rot = 1.0;
        }
      }

      // CALCULATE DISTANCE BETWEEN ROBOT AND HUMAN
      double distance = sqrt((robot_x - human_x)*(robot_x - human_x) + (robot_y - human_y)*(robot_y - human_y));

      // DECLARE MOTOR POWER MESSAGE
      std_msgs::Bool motor_msg;
      motor_msg.data = true; // SET MOTOR TO BE POWERED ON

      // IF MR IS WITHIN 0.5 METERS OF HUMAN, CUT MOTOR POWER
      if (distance <= 0.5)
      {
        motor_msg.data = false;
      }

      // PUBLISH MOTOR POWER DATA
      motor_pub.publish(motor_msg);

      // CLEAR PREVIOUS PARAMETERS
      conf.doubles.clear();

      // SET THE MAXIMUM FORWARD LINEAR SPEED OF THE ROBOT
      double_param.name = "max_vel_x";
      double_param.value = current_max_lin;
      conf.doubles.push_back(double_param); // ADD PARAMETER TO CONFIG LIST

      // SET THE MINIMUM LINEAR SPEED
      double_param.name = "min_vel_x";
      if (allow_back)
      {
        // IF BACKWARD MOTION IS ALLOWED, KEEP THE BACKWARD SPEED MATCHED TO THE CURRENT SAFE SPEED
        if (current_max_lin <= 0.13)
          double_param.value = -current_max_lin;
        else
          double_param.value = -0.13;
      }
      else
      {
        // IF BACKWARD MOTION IS NOT ALLOWED, KEEP MINIMUM SPEED AT 0
        double_param.value = 0.0;
      }
      conf.doubles.push_back(double_param); // ADD PARAMETER TO CONFIG LIST

      // SET THE MAXIMUM ROTATIONAL ANGULAR SPEED OF THE ROBOT
      double_param.name = "max_rot_vel";
      double_param.value = current_max_rot;
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

      // PUBLISH COSTUM MESSAGE
      lab9::behavior_params param_msg;
      param_msg.max_linear_speed = current_max_lin;
      param_msg.allow_backward = allow_back;
      param_msg.max_rotational_speed = current_max_rot;
      param_msg.adjust_orientation = adjust_orient;
      param_pub.publish(param_msg);

      // CHECK NAVIGATION RESULT WHILE MOVING
      if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        ROS_WARN("Failed point %d", i+1);
        break;
      }

      loop_rate.sleep(); // DELAY LOOP
    }

    // CHECK NAVIGATION RESULT
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Reached point %d", i+1);
    }
    else
    {
      ROS_WARN("Failed point %d", i+1);
    }

    i++; // INCREMENT POSITION VARIABLE WHEN ONE POSITION IS REACHED
  }

  // TURN MOTOR POWER BACK ON AT THE END OF THE PROGRAM
  std_msgs::Bool motor_msg;
  motor_msg.data = true;
  motor_pub.publish(motor_msg);

  infile.close(); // CLOSE parameters.txt FILE
  ROS_INFO("Done."); // END PROGRAM

  return 0;
}