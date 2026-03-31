// NAMES: Jordi Fraile Raton, Valentina Parodi
// CLASS: EGRS372
// PROJECT: Lab 7
// PURPOSE: Develop a mobile robot application requiring navigation on a map between pick and place locations, battery monitoring, and recharging at home position

// INCLUDE/DEFINE NECESSARY CONSTANTS/LIBRARIES
#include "ros/ros.h"                		 // INCLUDE ROS (ROBOT OPERATING SYSTEM)
#include <sstream>                  		 // LIBRARY THAT PROVIDES STRING CLASSES
#include <iostream>                 		 // FOR INPUTS AND OUTPUTS FOR CIN (INPUT)
#include "math.h"                   		 // FOR ABSOLUTE VALUE AND TRIG FUNCTIONS
#include <string>		    		 // FOR STRING DECLARATIONS
#include "tf/tfMessage.h"           		 // SUBSCRIBING TO TF/TFMESSAGE ...
#include "std_msgs/Byte.h"          		 // FOR READING THE PUSHBUTTON / BUMPER TOPIC
#include "std_msgs/Int32.h"          		 // FOR SENDING DATA TO THE 7-SEGMENT DISPLAY
#include "geometry_msgs/PoseStamped.h"	         // PUBLISHING A GEOMETRY_MSGS/...
#include "sensor_msgs/BatteryState.h"            // FOR READING THE BATTERY STATE TOPIC
#include "move_base_msgs/MoveBaseActionResult.h" // FOR READING move_base RESULT STATUS
#define INITIALIZE_VALUE -1                      // VARIABLES INITIALLY HOLD VALUE OF (-1)
#define PI 3.14159265359                         // RATIO OF CIRCLE, CIRCUMFERENCE:DIAMETER

// DEFINING GLOBAL VARIABLES
int count_placed = 0; // COUNTS NUMBER OF COMPLETED PLACE OPERATIONS 

// FLAGS
bool flag_pressed = false; // INDICATE PUSH BUTTON HAS BEEN PRESSED
bool flag_low_battery = false; // INDICATES LOW BATTERY
bool flag_goal_reached = false; // INDICATES DESIRED POSITION REACHED

// BATTERY
double battery_voltage = 12.0; // GOOD BATTERY LEVEL

// FIXED POSITIONS ON THE MAP
// HOME POSITION
double x_home = 1.0;
double y_home = 0.5;

// PICK POSITION
double x_pick = 3.0;
double y_pick = 0.5;

// PLACE POSITION
double x_place = 5.0;
double y_place = 0.8;

double psi = 0.0; // INITIALIZE ORIENTATION TO 0

// INITIALIZE RESULT MESSAGE
move_base_msgs::MoveBaseActionResult goalresult;

// STATE VARIABLE FOR CURRENT JOB
int current_job = 0;

int previous_job = 1; // INITIALIZE PREVIOUS JOB TO 1

// FUNCTION THAT HANDLES THE PUSHBUTTON INPUT
void pushbutton(const std_msgs::Byte::ConstPtr& cvalue)
{
  if (cvalue->data == 0)
  {
    flag_pressed = false; // PUSHBUTTON NOT PRESSED
  }
  else
  {
    flag_pressed = true; // PUSHBUTTON PRESSED
    ROS_INFO("Button pressed.");
  }
}

// FUNCTION THAT HANDLES THE BATTERY INPUT
void battery_read(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  battery_voltage = msg->voltage; // DECLARE BATTERY VOLTAGE 

  if (battery_voltage < 10.0)
  {
    flag_low_battery = true; // LOW BATTERY VOLTAGE
  }
  else if (battery_voltage > 11.0)
  {
    flag_low_battery = false; // GOOD BATTERY VOLTAGE
  }
}

// FUNCTION THAT HANDLES move_base/result TO INDICATE WHEN ROBOT HAS REACHED ONE OF THE FIXED POSITIONS 
void goal_result_read(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  goalresult = *msg; // DECLARE GOAL RESULT

  if (goalresult.status.status == 3)
  {
    flag_goal_reached = true; // IF EQUAL TO 3, THEN POSITION IS REACHED BY THE ROBOT
  }
}

// FUNCTION THAT SENDS A GOAL POSITION TO ROBOT
void send_goal(geometry_msgs::PoseStamped &goal, ros::Publisher &pub, double x, double y, double psi_angle)
{
  double z, w; // DECLARE QUATERNION VARIABLES
  
  // CALCULATE QUATERNIONS z AND w
  z = sin(psi_angle / 2.0 * PI / 180.0);
  w = cos(psi_angle / 2.0 * PI / 180.0);

  // ASSIGN FRAME
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();

  // ASSIGN POSITIONS BASED ON ROBOT DESTINATION
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;

  // ASSIGN ORIENTATIONS BASED ON ROBOT DESTINATION
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = z;
  goal.pose.orientation.w = w;

  flag_goal_reached = false; // SET FLAG TO FALSE TO INDICATE POSITION HAS NOT BEEN REACHED  
  pub.publish(goal); // PUBLISH DATA FOR DESIRED POSE
}

// MAIN PROGRAM - EXECUTE NAVIGATION BETWEEN PICK AND PLACE LOCATIONS AND GO HOME IF BATTERY IS LOW
int main(int argc, char **argv)
{
  // DECLARE VARIABLE FOR FRAME/POSE
  geometry_msgs::PoseStamped goal;

  // NAMES THE PROGRAM (FOR VISUAL PURPOSES)
  ros::init(argc, argv, "lab7");

  // NODEHANDLE::ADVERTISE() RETURNS A ROS::PUBLISHER OBJECT, 
  // WHICH SERVES TWO PURPOSES: 1) IT CONTAINS A PUBLISH() METHOD THAT 
  // LETS YOU PUBLISH MESSAGES ONTO THE TOPIC IT WAS CREATED WITH, AND 
  // 2) WHEN IT GOES OUT OF SCOPE, IT WILL AUTOMATICALLY UNADVERTISE.  
  ros::NodeHandle n;
  ros::NodeHandle n_sensor;
  ros::NodeHandle n_display;

  // PUBLISHER DECLARATION FOR THE MR POSE
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 50);

  // PUBLISHER DECLARATION FOR THE 7-SEGMENT DISPLAY
  ros::Publisher LED_pub = n_display.advertise<std_msgs::Int32>("LED", 100);

  // SUBSCRIBES TO BUMPER/PUSHBUTTON TOPIC
  ros::Subscriber bumper_sub = n_sensor.subscribe("bumper", 1, pushbutton);
  // SUBSCRIBES TO BATTERY STATE TOPIC
  ros::Subscriber battery_sub = n_sensor.subscribe("battery_state", 1, battery_read);
  // SUBSCRIBES TO move_base RESULT TOPIC
  ros::Subscriber result_sub = n_sensor.subscribe("move_base/result", 1, goal_result_read);
  
  // SETS THE FREQUENCY FOR WHICH THE PROGRAM SLEEPS AT
  ros::Rate loop_rate(20);

  // INITIALIZE THE 7-SEGMENT DISPLAY TO ZERO
  std_msgs::Int32 led_display_msg;
  led_display_msg.data = count_placed;
  LED_pub.publish(led_display_msg);
  ros::spinOnce();
  loop_rate.sleep();

  current_job = 0; // CURRENT JOB = GO HOME

  // START BY GOING HOME
  while(flag_goal_reached == false)
  {
    send_goal(goal, pub, x_home, y_home, psi); // SEND ROBOT TO HOME POSITION BY CALLING THE send_goal FUNCTION
    ROS_INFO("MR going home...");
    ros::spinOnce();
  }
  
  // PERFORM NAGIVATION BETWEEN PICK AND PLACE LOCATIONS 
  while (ros::ok())
  {
    ros::spinOnce();

    // LOW BATTERY CONDITION
    if (flag_low_battery && current_job != 5)
    {
      previous_job = current_job; // STORE THE JOB THE ROBOT WAS DOING BEFORE GOING HOME
      current_job = 5; // SET CURRENT JOB TO CHARGING
      send_goal(goal, pub, x_home, y_home, psi); // SEND ROBOT TO HOME POSITION BY CALLING THE send_goal FUNCTION
      ROS_INFO("Battery low. Robot returning home...");
    }

    // GO HOME AT START
    if (current_job == 0)
    {
      if (flag_goal_reached)
      {
        ROS_INFO("MR at home...");
        current_job = 1; // UPDATE CURRENT JOB TO GO TO PICK POSITION
        send_goal(goal, pub, x_pick, y_pick, psi); // SEND ROBOT TO PICK POSITION BY CALLING THE send_goal FUNCTION
        ROS_INFO("MR going to pick point...");
      }
    }

    // GO TO PICK
    else if (current_job == 1)
    {
      if (flag_goal_reached)
      {
        ROS_INFO("MR at pick point. Waiting for button...");
        current_job = 3; // UPDATE CURRENT JOB TO WAIT AT PICK POSITION
      }
    }

    // WAIT AT PICK
    else if (current_job == 3)
    {
      if (flag_pressed)
      {
        flag_pressed = false; // RESET BUTTON FLAG AFTER DETECTING PRESS
        current_job = 2; // UPDATE CURRENT JOB TO GO TO PLACE POSITION
        send_goal(goal, pub, x_place, y_place, psi); // SEND ROBOT TO PLACE POSITION BY CALLING THE send_goal FUNCTION
        ROS_INFO("MR going to place...");
      }
    }

    // GO TO PLACE
    else if (current_job == 2)
    {
      if (flag_goal_reached)
      {
        ROS_INFO("MR at place point. Waiting for button...");
        current_job = 4; // UPDATE CURRENT JOB TO WAIT AT PLACE POSITION
      }
    }

    // WAIT AT PLACE
    else if (current_job == 4)
    {
      if (flag_pressed)
      {
        flag_pressed = false; // RESET BUTTON FLAG AFTER DETECTING PRESS
        count_placed = count_placed + 1; // INCREMENT NUMBER OF COMPLETED PLACE OPERATIONS

        led_display_msg.data = count_placed; // UPDATE DISPLAY MESSAGE WITH NEW COUNT
        LED_pub.publish(led_display_msg); // PUBLISH NEW COUNT TO 7-SEGMENT DISPLAY

        ROS_INFO("Sequence completed.");

        current_job = 1; // UPDATE CURRENT JOB TO RETURN TO PICK POSITION
        send_goal(goal, pub, x_pick, y_pick, psi); // SEND ROBOT TO PICK POSITION BY CALLING THE send_goal FUNCTION
        ROS_INFO("MR going back to pick...");
      }
    }

    // CHARGING AT HOME
    else if (current_job == 5)
    {
      if (!flag_low_battery && flag_goal_reached)
      {
        ROS_INFO("Battery charged. Returning to previous job...");

        if (previous_job == 1 || previous_job == 3)
        {
          current_job = 1; // RETURN TO PICK JOB
          send_goal(goal, pub, x_pick, y_pick, psi); // SEND ROBOT TO PICK POSITION BY CALLING THE send_goal FUNCTION
          ROS_INFO("MR going back to pick...");
        }
        else if (previous_job == 2 || previous_job == 4)
        {
          current_job = 2; // RETURN TO PLACE JOB
          send_goal(goal, pub, x_place, y_place, psi); // SEND ROBOT TO PLACE POSITION BY CALLING THE send_goal FUNCTION
          ROS_INFO("MR going back to place...");
        }
        else
        {
          current_job = 1; // DEFAULT TO PICK JOB IF PREVIOUS JOB IS NOT RECOGNIZED
          send_goal(goal, pub, x_pick, y_pick, psi); // SEND ROBOT TO PICK POSITION BY CALLING THE send_goal FUNCTION
          ROS_INFO("MR going to pick...");
        }
      }
    }
    loop_rate.sleep();
  }
  return 0;
}
