// NAMES: Jordi Fraile Raton, Valentina Parodi
// CLASS: EGRS372
// PROJECT: Lab 6
// PURPOSE: Create a mobile robot application using the pushbutton and 7-segment LED

// INCLUDE/DEFINE NECESSARY CONSTANTS/LIBRARIES
#include "ros/ros.h"              // INCLUDE ROS (ROBOT OPERATING SYSTEM)
#include <sstream>                // LIBRARY THAT PROVIDES STRING CLASSES
#include <iostream>               // FOR INPUTS AND OUTPUTS FOR CIN (INPUT)
#include "math.h"                 // FOR ABSOLUTE VALUE AND TRIG FUNCTIONS
#include "geometry_msgs/Twist.h"  // PUBLISHING A GEOMETRY_MSGS/TWIST ...
#include "tf/tfMessage.h"         // SUBSCRIBING TO TF/TFMESSAGE ...
#include "std_msgs/Byte.h"        // FOR READING THE PUSHBUTTON / BUMPER TOPIC
#include "std_msgs/Int32.h"       // FOR SENDING DATA TO THE 7-SEGMENT DISPLAY
#define INITIALIZE_VALUE -1       // VARIABLES INITIALLY HOLD VALUE OF (-1)
#define PI 3.14159265359          // RATIO OF CIRCLE, CIRCUMFERENCE:DIAMETER

// DEFINING GLOBAL VARIABLES 
bool flag;                              // 'FLAG' = FALSE WHEN CONDITIONS ARE MET
double current_angle = 0;               // CURRENT ANGLE OF THE ROBOT
double target_speed = 0.0;              // ROBOT TRAVELLING SPEED (METERS/SECOND OR RAD/SECOND)
double target_angle = INITIALIZE_VALUE; // DESIRED ANGLE OF ROBOT PATH (THETA)
double target_forward = 0.35;           // DISTANCE THE ROBOT IS TRYING TO MOVE WHEN BACKING UP
double moved = 0;                       // DISTANCE TRAVELLED (IN METERS)
double initialx = INITIALIZE_VALUE;     // INITIAL X COORDINATE OF A MOVE
double initialy = INITIALIZE_VALUE;     // INITIAL Y COORDINATE OF A MOVE
int wall_count = 0;                     // NUMBER OF TIMES THE PUSHBUTTON HAS BEEN ACTIVATED

// THIS FUNCTION HANDLES THE PUSHBUTTON INPUT ---------------------------------
// IF THE BUTTON IS PRESSED, THE WALL CONTACT COUNT IS INCREASED --------------
// AND THE FORWARD MOTION LOOP IS TERMINATED ----------------------------------
// ACCEPTS std_msgs::Byte cvalue (CONSTANT) -----------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void pushbutton(const std_msgs::Byte cvalue)
{
  if(cvalue.data == 0)
  {
    wall_count = wall_count;
  }
  else
  {
    wall_count = wall_count + 1;
    std::cout << "Wall Contact Count: " << wall_count << std::endl;
    flag = false;
  }
}

// THIS FUNCTION HANDLES BACKWARD MOVEMENT OF THE ROBOT -----------------------
// WILL CHECK TO SEE IF THE DISTANCE MOVED IS LESS THAN THE TARGET ------------
// ACCEPTS tf::tfMessage cvalue (CONSTANT) ------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void backwardprog(const tf::tfMessage cvalue)
{
  double dx, dy; // VARIABLES FOR X,Y COORDINATES

  // SETS THE INITIAL X AND Y COORDINATES
  if(initialx == INITIALIZE_VALUE || initialy == INITIALIZE_VALUE)
  {
    initialx = cvalue.transforms[0].transform.translation.x;
    initialy = cvalue.transforms[0].transform.translation.y;
  }

  // CALCULATES DISTANCE IN X AND Y TRAVELED
  dx = std::abs(cvalue.transforms[0].transform.translation.x - initialx);
  dy = std::abs(cvalue.transforms[0].transform.translation.y - initialy);
  
  // CALCULATES TOTAL DISTANCE THE ROBOT HAS TRAVELLED
  moved = sqrt(dx*dx + dy*dy);
  
  // CALCULATES DISTANCE (METERS) THE ROBOT HAS YET TO TRAVEL
  if(moved > target_forward)
  {
    flag = false;
  }

  // SETS A SPEED PROPORTIONAL TO THE DISTANCE YET TO BE TRAVELED
  // PLUS AN OFFSET TO ACCOUNT FOR FRICTION
  // SPEED IS IN M/S
  target_speed = std::abs(target_forward - moved)/4 + 0.1;
}

// THIS FUNCTION HANDLES THE 90 DEGREE TURN OF THE ROBOT ----------------------
// ACCEPTS tf::tfMessage cvalue (CONSTANT) ------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void Turnprog(const tf::tfMessage cvalue)
{
  double turnz, turnw, mindist;

  // CALCULATE ORIENTATION OF THE ROBOT
  turnz = cvalue.transforms[0].transform.rotation.z;
  turnw = cvalue.transforms[0].transform.rotation.w;

  // CALCULATE CURRENT ANGLE OF THE ROBOT
  current_angle = 2 * atan2(turnz, turnw);

  // CONVERTS THE CURRENT ANGLE TO BE BETWEEN 0 AND 2PI
  if(current_angle < 0)
  {
    current_angle = current_angle + 2 * PI;
  }
  if(current_angle >= 2 * PI)
  {
    current_angle = current_angle - 2 * PI;
  }
  
  // SETS THE TARGET ANGLE
  if(target_angle == INITIALIZE_VALUE)
  {
    target_angle = current_angle + (PI / 2.0);
  }
  
  // CONVERTS THE TARGET ANGLE TO BE BETWEEN 0 AND 2PI
  if(target_angle < 0)
  {
    target_angle = target_angle + 2 * PI;
  }
  if(target_angle >= 2 * PI)
  {
    target_angle = target_angle - 2 * PI;
  }

  // DETERMINES IF THE ROBOT HAS PASSED THE TARGET ANGLE. 
  // (ONLY WORKS FOR TURNING COUNTER-CLOCKWISE)
  // THE LOGIC AFTER && ACCOUNTS FOR GOING FROM A HIGH CURRENT ANGLE,
  // SUCH AS 7PI/4, TO A LOW VALUE, SUCH AS 0
  if((current_angle>=target_angle)&&(std::abs(current_angle-target_angle)<0.1))
  {
    flag = false;
  }

  // FINDS THE MINIMUM ANGLE BETWEEN THE CURRENT AND TARGET ANGLE
  mindist = std::abs(target_angle-current_angle);
  
  // ACCOUNTS FOR GOING FROM A HIGH CURRENT ANGLE, 
  // SUCH AS 7PI/4, TO A LOW VALUE, SUCH AS 0
  if(std::abs(target_angle-current_angle+2*PI)<mindist)
  {
    mindist=std::abs(target_angle-current_angle+2*PI);
  }

  // SETS THE TARGET SPEED TO BE PROPORTIONAL TO THE NEEDED ANGLE FOR TRAVEL
  // PLUS AN OFFSET TO ACCOUNT FOR FRICTION
  // SPEED IS MEASURED IN RAD/S
  target_speed = mindist / PI + 0.5;
}

// MAIN PROGRAM - MOVES FORWARD UNTIL WALL HIT, THEN BACKS UP, TURNS 90 DEGREES, REPEATS
int main(int argc, char **argv)
{
  // NAMES THE PROGRAM (FOR VISUAL PURPOSES)
  ros::init(argc, argv, "lab6");
  
  // NODEHANDLE::ADVERTISE() RETURNS A ROS::PUBLISHER OBJECT, 
  // WHICH SERVES TWO PURPOSES: 1) IT CONTAINS A PUBLISH() METHOD THAT 
  // LETS YOU PUBLISH MESSAGES ONTO THE TOPIC IT WAS CREATED WITH, AND 
  // 2) WHEN IT GOES OUT OF SCOPE, IT WILL AUTOMATICALLY UNADVERTISE.
  ros::NodeHandle n;
  ros::NodeHandle n_sensor;
  ros::NodeHandle n_display;

  // PUBLISHER DECLARATION FOR THE VELOCITY OF THE ROBOT
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // PUBLISHER DECLARATION FOR THE 7-SEGMENT DISPLAY
  ros::Publisher LED_pub = n_display.advertise<std_msgs::Int32>("LED", 100);
  
  ros::Subscriber tf; // SUBSCRIBER DECLARATION TO GET THE TRANSFORMATION MATRICES OF THE ROBOT
  
  // SETS THE FREQUENCY FOR WHICH THE PROGRAM SLEEPS AT
  ros::Rate loop_rate(100);
  
  // GETS AN INPUT FROM THE USER, WAITS UNTIL IT IS 'Y'
  char input = 0;
  fflush(stdin);
  std::cout << "Type 'y' and enter to begin: ";
  while(input != 'y' && ros::ok())
  {
    std::cin >> input;
  }

  // INITIALIZES THE ROBOT VELOCITY
  geometry_msgs::Twist c;
  c.linear.x = 0.0;
  c.linear.y = 0.0;
  c.linear.z = 0.0;
  c.angular.x = 0.0;
  c.angular.y = 0.0;
  c.angular.z = 0.0;
  
  // INITIALIZES THE 7-SEGMENT DISPLAY TO ZERO
  std_msgs::Int32 led_display_msg;
  led_display_msg.data = wall_count;
  LED_pub.publish(led_display_msg);
  ros::spinOnce();
  loop_rate.sleep();
  
  // CREATES A TIME VARIABLE
  ros::Time go;
  
  // LOOPS INFINITE TIMES TIL USER SHUTDOWN
  while(ros::ok())
  {
    // INITIALIZES THE FLAG FOR THE FORWARD MOTION SECTION
    flag = true;

    // SUBSCRIBES TO THE BUMPER TOPIC USING THE PUSHBUTTON FUNCTION
    ros::Subscriber bumper_sub = n_sensor.subscribe("bumper", 1, pushbutton);
    ros::spinOnce();
    loop_rate.sleep();
    
    // DRIVES FORWARD UNTIL THE PUSHBUTTON IS ACTIVATED
    while(flag && ros::ok())
    {
      c.linear.x = 0.10;
      c.angular.z = 0.0;
      vel_pub.publish(c);

      ros::spinOnce();
      loop_rate.sleep();
    }

    // SHUTS DOWN THE SUBSCRIBER NODE TO CHANGE ITS RELATED FUNCTION LATER
    bumper_sub.shutdown();

    // STOPS THE ROBOT FOR A LITTLE TO BE SAFE
    go = ros::Time::now();
    while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
    {
      c.linear.x = 0.0;
      c.angular.z = 0.0;
      vel_pub.publish(c);

      ros::spinOnce();
      loop_rate.sleep();
    }
	
    // UPDATES THE 7-SEGMENT DISPLAY WITH THE CURRENT WALL COUNT
    led_display_msg.data = wall_count;
    LED_pub.publish(led_display_msg);
    ros::spinOnce();
    loop_rate.sleep();

    // INITIALIZES THE VALUES NEEDED FOR BACKWARD MOTION
    flag = true;
    moved = 0;
    initialx = INITIALIZE_VALUE;
    initialy = INITIALIZE_VALUE;

    // SUBSCRIBES TO THE TF NODE WITH THE FUNCTION BACKWARDPROG
    tf = n.subscribe("/tf", 1, backwardprog);
    ros::spinOnce();
    loop_rate.sleep();

    // BACKS UP UNTIL THE NECESSARY DISTANCE IS TRAVELED
    while(flag && ros::ok())
    {
      c.linear.x = -target_speed;
      c.angular.z = 0.0;
      vel_pub.publish(c);

      ros::spinOnce();
      loop_rate.sleep();
    }
    
    // SHUTS DOWN THE SUBSCRIBER NODE TO CHANGE ITS RELATED FUNCTION LATER
    tf.shutdown();

    // STOPS THE ROBOT FOR A LITTLE TO BE SAFE
    go = ros::Time::now();
    while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
    {
      c.linear.x = 0.0;
      c.angular.z = 0.0;
      vel_pub.publish(c);

      ros::spinOnce();
      loop_rate.sleep();
    }
	
    // INITIALIZES THE FLAG FOR TURNING
    flag = true;
    target_angle = INITIALIZE_VALUE;

    // SUBSCRIBES TO THE TF NODE WITH THE FUNCTION TURNPROG
    tf = n.subscribe("/tf", 1, Turnprog);
    ros::spinOnce();
    loop_rate.sleep();

    // SPINS IN PLACE COUNTER CLOCKWISE UNTIL THE DESIRED ANGLE IS REACHED
    while(flag && ros::ok())
    {
      c.linear.x = 0.0;
      c.angular.z = target_speed;
      vel_pub.publish(c);

      ros::spinOnce();
      loop_rate.sleep();
    }
    
    // SHUTS DOWN THE SUBSCRIBER NODE TO CHANGE ITS RELATED FUNCTION LATER
    tf.shutdown();

    // INCREMENTS THE TARGET ANGLE
    target_angle=target_angle+PI/2;

    // STOPS THE ROBOT FOR A LITTLE TO BE SAFE
    go = ros::Time::now();
    while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
    {
      c.linear.x = 0.0;
      c.angular.z = 0.0;
      vel_pub.publish(c);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  
  // STOPS THE ROBOT BEFORE ENDING THE PROGRAM
  c.linear.x = 0.0;
  c.angular.z = 0.0;
  vel_pub.publish(c);
  ros::spinOnce();
  loop_rate.sleep();
 
  return 0;
}
