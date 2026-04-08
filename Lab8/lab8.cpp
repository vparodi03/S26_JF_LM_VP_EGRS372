// NAMES: Jordi Fraile Raton, Valentina Parodi
// CLASS: EGRS372
// PROJECT: Lab 8
// PURPOSE: Create launch files, custom messages, and services in ROS and develop a MR application, where the user can monitor the status of a MR and interact with simple commands 

// INCLUDE/DEFINE NECESSARY CONSTANTS/LIBRARIES
#include "ros/ros.h"      // INCLUDE ROS (ROBOT OPERATING SYSTEM)
#include <sstream>        // LIBRARY THAT PROVIDES STRING CLASSES
#include <iostream>       // FOR INPUTS AND OUTPUTS FOR CIN (INPUT)
#include "math.h"         // FOR ABSOLUTE VALUE AND TRIG FUNCTIONS
#include <string>         // FOR STRING DECLARATIONS
#include "geometry_msgs/PoseStamped.h"              // PUBLISHING A GEOMETRY_MSGS/...
#include "std_msgs/Byte.h"                          // FOR READING THE PUSHBUTTON / BUMPER TOPIC
#include "std_msgs/Int32.h"                         // FOR SENDING DATA TO THE 7-SEGMENT DISPLAY
#include "sensor_msgs/BatteryState.h"               // FOR READING THE BATTERY STATE TOPIC
#include "move_base_msgs/MoveBaseActionResult.h"    // FOR READING move_base RESULT STATUS
#define PI 3.14159265359                            // RATIO OF CIRCLE, CIRCUMFERENCE:DIAMETER

// COSTUM MESSAGE AND SERVICE HEADERS
#include "lab7/turtlebot_status.h"
#include "lab7/go_home.h"
#include "lab7/return_to_work.h"
#include "lab7/update_count.h"

// DEFINING GLOBAL VARIABLES
int places = 0; // COUNTS NUMBER OF COMPLETED PLACE OPERATIONS

// FLAGS
bool flag_pressed = false; // INDICATE PUSH BUTTON HAS BEEN PRESSED
bool flag_low_battery = false; // INDICATES LOW BATTERY
bool flag_goal_reached = false; // INDICATES DESIRED POSITION REACHED

// BATTERY
double battery_voltage; // STORES CURRENT BATTERY VOLTAGE
double battery_low; // STORES LOW BATTERY THRESHOLD
double battery_high; // STORES HIGH BATTERY THRESHOLD

// POSITION VARIABLES ON THE MAP
double x_home, y_home; // HOME POSITION
double x_pick, y_pick; // PICK POSITION 
double x_place, y_place; // PLACE POSITION
double psi_home, psi_pick, psi_place; // ORIENTATION AT EACH POSITION

// INITIALIZE RESULT MESSAGE
move_base_msgs::MoveBaseActionResult goalresult;

// STATE VARIABLES
int current_job = 0;
int previous_job = 1; // INITIALIZE PREVIOUS JOB TO 1

// PUBLISHERS
ros::Publisher pub; // PUBLISHER FOR MR GOAL POSE
ros::Publisher LED_pub; // PUBLISHER FOR 7-SEGMENT DISPLAY
ros::Publisher status_pub; // PUBLISHER FOR CUSTOM STATUS MESSAGE

// GOAL
geometry_msgs::PoseStamped goal; // VARIABLE USED TO SEND ROBOT GOAL

// JOB DIVISION BASED ON VARIABLE VALUE
std::string job_name(int job)
{
  if (job == 0)
  {
    return "going_home_init";
  }
  else if (job == 1)
  {
    return "going_to_pick";
  }
  else if (job == 2)
  {
    return "going_to_place";
  }
  else if (job == 3)
  {
    return "waiting_at_pick";
  }
  else if (job == 4)
  {
    return "waiting_at_place";
  }
  else if (job == 5)
  {
    return "charging_at_home";
  }
  else if (job == 6)
  {
    return "manually_sent_home";
  }
  else
  {
    return "unknown";
  }
}

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

  if (battery_voltage < battery_low)
  {
    flag_low_battery = true; // LOW BATTERY VOLTAGE
  }
  else if (battery_voltage > battery_high)
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

// FUNCTION THAT CALLS THE SERVICE AND UPDATES THE COUNT 
bool update_count(lab7::update_count::Request &req, lab7::update_count::Response &res)
{
  std_msgs::Int32 LED_update; // DECLARE VARIABLE FOR 7-SEGMENT DISPLAY

  // SET RESPONSE TO THE OLD NUMBER OF PLACES
  res.old_count = places;
  // UPDATE THE NUMBER OF PLACES WITH THE REQUEST
  places = req.new_count;
  // UPDATE THE MESSAGE VARIABLE WITH THE NEW NUMBER
  LED_update.data = places;
  // PUBLISH THE NEW LED DATA
  LED_pub.publish(LED_update);

  ROS_INFO("Old count: %ld, New count: %ld", res.old_count, req.new_count);
  return true; // RETURN TRUE IF SERVICE EXECUTES SUCCESSFULLY
}

// FUNCTION THAT CALLS THE SERVICE AND SENDS THE ROBOT HOME 
bool go_home_service(lab7::go_home::Request &req, lab7::go_home::Response &res)
{
  res.old_job = current_job; // RETURN OLD JOB
  previous_job = current_job; // SAVE CURRENT JOB TO RETURN TO WORK LATER
  current_job = 6; // SET CURRENT JOB TO MANUALLY SENT HOME
  send_goal(goal, pub, x_home, y_home, psi_home); // SEND ROBOT TO HOME POSITION
 
  ROS_INFO("Sending robot home. Previous job: %d", previous_job);
  return true; // RETURN TRUE IF SERVICE EXECUTES SUCCESSFULLY
}

// FUNCTION THAT CALLS THE SERVICE AND SENDS THE ROBOT BACK TO ITS CURRENT JOB
bool return_to_work_service(lab7::return_to_work::Request &req, lab7::return_to_work::Response &res)
{
  res.old_job = current_job; // RETURN CURRENT JOB

  if (previous_job == 1 || previous_job == 3)
  {
    current_job = 1; // RETURN TO PICK JOB
    send_goal(goal, pub, x_pick, y_pick, psi_pick); // SEND ROBOT TO PICK POSITION
    ROS_INFO("Returning to pick point.");
  }
  else if (previous_job == 2 || previous_job == 4)
  {
    current_job = 2; // RETURN TO PLACE JOB
    send_goal(goal, pub, x_place, y_place, psi_place); // SEND ROBOT TO PLACE POSITION
    ROS_INFO("Returning to place point.");
  }
  else
  {
    current_job = 1; // DEFAULT TO PICK JOB
    send_goal(goal, pub, x_pick, y_pick, psi_pick); // SEND ROBOT TO PICK POSITION
    ROS_INFO("Defaulting to pick point.");
  }
  return true; // RETURN TRUE IF SERVICE EXECUTES SUCCESSFULLY
}

// MAIN PROGRAM: EXECUTE MR APPLICATION 
int main(int argc, char **argv)
{
  // NAMES THE PROGRAM (FOR VISUAL PURPOSES)
  ros::init(argc, argv, "lab7");

  // NODEHANDLE::ADVERTISE() RETURNS A ROS::PUBLISHER OBJECT,
  // WHICH SERVES TWO PURPOSES: 1) IT CONTAINS A PUBLISH() METHOD THAT
  // LETS YOU PUBLISH MESSAGES ONTO THE TOPIC IT WAS CREATED WITH, AND
  // 2) WHEN IT GOES OUT OF SCOPE, IT WILL AUTOMATICALLY UNADVERTISE.
  ros::NodeHandle n;
  ros::NodeHandle n_sensor;
  ros::NodeHandle n_display;

  // PUBLISHERS
  pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 50); // PUBLISHER FOR THE MR POSE
  LED_pub = n_display.advertise<std_msgs::Int32>("LED", 100); // PUBLISHER FOR THE 7-SEGMENT DISPLAY
  status_pub = n.advertise<lab7::turtlebot_status>("TurtleBot_Status", 1); // PUBLISHER FOR CUSTOM STATUS MESSAGE

  // SUBSCRIBERS
  ros::Subscriber bumper_sub = n_sensor.subscribe("bumper", 1, pushbutton); // SUBSCRIBES TO BUMPER/PUSHBUTTON TOPIC
  ros::Subscriber battery_sub = n_sensor.subscribe("battery_state", 1, battery_read); // SUBSCRIBES TO BATTERY STATE TOPIC
  ros::Subscriber result_sub = n_sensor.subscribe("move_base/result", 1, goal_result_read); // SUBSCRIBES TO move_base RESULT TOPIC

  // SERVICES
  ros::ServiceServer count_service = n.advertiseService("update_count", update_count); // SERVICE TO UPDATE PLACE COUNT
  ros::ServiceServer home_service = n.advertiseService("go_home", go_home_service); // SERVICE TO SEND ROBOT HOME
  ros::ServiceServer work_service = n.advertiseService("return_to_work", return_to_work_service); // SERVICE TO RETURN ROBOT TO WORK

  // SETS THE FREQUENCY FOR WHICH THE PROGRAM SLEEPS AT
  ros::Rate loop_rate(20);

  // INITIALIZE THE 7-SEGMENT DISPLAY TO ZERO
  std_msgs::Int32 led_display_msg;
  led_display_msg.data = places;
  LED_pub.publish(led_display_msg);
  ros::spinOnce();
  loop_rate.sleep();

  // LOAD PARAMS BEFORE GOING HOME
  ros::param::get("/battery_low_level", battery_low); // GET LOW BATTERY THRESHOLD
  ros::param::get("/battery_high_level", battery_high); // GET HIGH BATTERY THRESHOLD
  ros::param::get("/home_location/x", x_home); // GET HOME X POSITION
  ros::param::get("/home_location/y", y_home); // GET HOME Y POSITION
  ros::param::get("/home_location/theta", psi_home); // GET HOME ORIENTATION
  ros::param::get("/pick_location/x", x_pick); // GET PICK X POSITION
  ros::param::get("/pick_location/y", y_pick); // GET PICK Y POSITION
  ros::param::get("/pick_location/theta", psi_pick); // GET PICK ORIENTATION
  ros::param::get("/place_location/x", x_place); // GET PLACE X POSITION
  ros::param::get("/place_location/y", y_place); // GET PLACE Y POSITION
  ros::param::get("/place_location/theta", psi_place); // GET PLACE ORIENTATION

  // START BY GOING HOME
  current_job = 0; // CURRENT JOB = GO HOME
  send_goal(goal, pub, x_home, y_home, psi_home); // SEND ROBOT TO HOME POSITION
  ROS_INFO("MR going home...");

  while (ros::ok())
  {
    // REFRESH PARAMS
    ros::param::get("/battery_low_level", battery_low); // REFRESH LOW BATTERY THRESHOLD
    ros::param::get("/battery_high_level", battery_high); // REFRESH HIGH BATTERY THRESHOLD
    ros::param::get("/home_location/x", x_home); // REFRESH HOME X POSITION
    ros::param::get("/home_location/y", y_home); // REFRESH HOME Y POSITION
    ros::param::get("/pick_location/x", x_pick); // REFRESH PICK X POSITION
    ros::param::get("/pick_location/y", y_pick); // REFRESH PICK Y POSITION
    ros::param::get("/place_location/x", x_place); // REFRESH PLACE X POSITION
    ros::param::get("/place_location/y", y_place); // REFRESH PLACE Y POSITION
    ros::param::get("/home_location/theta", psi_home); // REFRESH HOME ORIENTATION
    ros::param::get("/pick_location/theta", psi_pick); // REFRESH PICK ORIENTATION
    ros::param::get("/place_location/theta", psi_place); // REFRESH PLACE ORIENTATION

    ros::spinOnce();

    // PUBLISH STATUS MESSAGE
    lab7::turtlebot_status status_msg;
    status_msg.current_job = job_name(current_job); // STORE CURRENT JOB
    status_msg.place_count = places; // STORE NUMBER OF COMPLETED PLACES
    status_msg.battery = battery_voltage; // STORE BATTERY VOLTAGE
    status_pub.publish(status_msg); // PUBLISH CUSTOM STATUS MESSAGE

    // LOW BATTERY CONDITION
    if (flag_low_battery && current_job != 5 && current_job != 6)
    {
      previous_job = current_job; // STORE THE JOB THE ROBOT WAS DOING BEFORE GOING HOME
      current_job = 5; // SET CURRENT JOB TO CHARGING
      send_goal(goal, pub, x_home, y_home, psi_home); // SEND ROBOT TO HOME POSITION
      ROS_INFO("Battery low. Robot returning home to charge...");
    }

    // GO HOME AT START
    if (current_job == 0)
    {
      if (flag_goal_reached)
      {
        ROS_INFO("MR at home.");
        current_job = 1; // UPDATE CURRENT JOB TO GO TO PICK POSITION
        send_goal(goal, pub, x_pick, y_pick, psi_pick); // SEND ROBOT TO PICK POSITION
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
        send_goal(goal, pub, x_place, y_place, psi_place); // SEND ROBOT TO PLACE POSITION
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
        places++; // INCREMENT NUMBER OF COMPLETED PLACE OPERATIONS

        led_display_msg.data = places; // UPDATE DISPLAY MESSAGE WITH NEW COUNT
        LED_pub.publish(led_display_msg); // PUBLISH NEW COUNT TO 7-SEGMENT DISPLAY

        ROS_INFO("Sequence completed.");

        current_job = 1; // UPDATE CURRENT JOB TO RETURN TO PICK POSITION
        send_goal(goal, pub, x_pick, y_pick, psi_pick); // SEND ROBOT TO PICK POSITION
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
          send_goal(goal, pub, x_pick, y_pick, psi_pick); // SEND ROBOT TO PICK POSITION
          ROS_INFO("MR going back to pick...");
        }
        else if (previous_job == 2 || previous_job == 4)
        {
          current_job = 2; // RETURN TO PLACE JOB
          send_goal(goal, pub, x_place, y_place, psi_place); // SEND ROBOT TO PLACE POSITION
          ROS_INFO("MR going back to place...");
        }
        else
        {
          current_job = 1; // DEFAULT TO PICK JOB IF PREVIOUS JOB IS NOT RECOGNIZED
          send_goal(goal, pub, x_pick, y_pick, psi_pick); // SEND ROBOT TO PICK POSITION
          ROS_INFO("MR going to pick...");
        }
      }
    }

    // MANUALLY SENT HOME (go_home SERVICE)
    else if (current_job == 6)
    {
      if (flag_goal_reached)
      {
        ROS_INFO("MR at home . Waiting for return_to_work service...");
        flag_goal_reached = false; // RESET FLAG SO MESSAGE DOES NOT REPEAT CONTINUOUSLY
      }
    }

    loop_rate.sleep();
  }
  return 0;
}