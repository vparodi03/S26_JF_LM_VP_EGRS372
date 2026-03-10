// NAMES: Jordi Fraile Raton, Valentina Parodi
// CLASS: EGRS372
// PROJECT: Lab 5
// PURPOSE: Develop a mobile robot application requiring navigation on a map which has been created using SLAM process

// Include libraries
#include "ros/ros.h"                    // INCLUDE ROS (ROBOT OPERATING SYSTEM) 
#include <sstream>                      // LIBRARY THAT PROVIDES STRING CLASSES
#include <iostream> 		        // FOR INPUTS AND OUTPUTS FOR CIN (INPUT)
#include "math.h"   		        // FOR ABSOLUTE VALUE AND TRIG FUNCTIONS
#include "geometry_msgs/PoseStamped.h"  // PUBLISHING A GEOETRY_MSGS/...
#include "geometry_msgs/Twist.h"        // PUBLISHING A GEOETRY_MSGS/...
#include "tf/tfMessage.h"  	        // SUBSCRIBING TO TF/TFMESSAGE ...
#define INITIALIZE_VALUE -1             // VARIABLES INITIALLY HOLD VALUE OF (-1)
#define PI 3.14159265359                // RATIO OF CIRCLE, CIRCUMFERENCE:DIAMETER
#include "std_msgs/UInt64.h"            // LIBRARY FOR INTEGER VALUES
#include <string>                       // LIBRARY FOR STRINGS

// Main Porgram: Enable TurtleBot to move autonomously to a specified user defined pose within the map 
int main(int argc, char **argv)
{
  // DECLARE REFERENCE FRAME AND VARIABLES
  std::string frame;
  double x, y, psi, z, w;  
  geometry_msgs::PoseStamped goal;

  // NAMES THE PROGRAM (FOR VISUAL PURPOSES)    
  ros::init(argc, argv, "map_navigation");

  // NODEHANDLE::ADVERTISE() RETURNS A ROS::PUBLISHER OBJECT, 
  // WHICH SERVES TWO PURPOSES: 1) IT CONTAINS A PUBLISH() METHOD THAT 
  // LETS YOU PUBLISH MESSAGES ONTO THE TOPIC IT WAS CREATED WITH, AND 
  // 2) WHEN IT GOES OUT OF SCOPE, IT WILL AUTOMATICALLY UNADVERTISE.    
  ros::NodeHandle n;

  // PUBLISHER DECLARATION FOR THE MR POSE
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 50);

  // SETS THE FREQUENCY FOR WHICH THE PROGRAM SLEEPS AT. 1000=1/1000 SECOND
  // A ROS::RATE OBJECT ALLOWS YOU TO SPECIFY A FREQUENCY THAT YOU WOULD LIKE TO 
  // LOOP AT. IT WILL KEEP TRACK OF HOW LONG IT HAS BEEN SINCE THE LAST CALL 
  // TO RATE::SLEEP(), AND SLEEP FOR THE CORRECT AMOUNT OF TIME. Ref:
  // http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    // ASK USER TO SELECT REFERENCE FRAME 
    fflush(stdin);
    std::cout << "Enter reference frame (map or base_link): ";
    std::cin >> frame;
   
    // ASK USER FOR DESTINATION IN X
    fflush(stdin);
    std::cout << "Enter x position: ";
    std::cin >> x;

    // ASK USER FOR DESTINATION IN Y
    fflush(stdin);
    std::cout << "Enter y position: ";
    std::cin >> y;

    // ASK USER FOR ROTATION ANGLE
    fflush(stdin);
    std::cout << "Enter rotation (psi in degrees): ";
    std::cin >> psi;

    // CALCULATE QUATERNION z
    z = sin(psi/2*PI/180.0);

    // CALCULATE QUATERNION w
    w = cos(psi/2*PI/180.0); 
      
    // ASSIGN FRAME
    goal.header.frame_id = frame;
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
   
    // PUBLISH DATA
    pub.publish(goal);
    ROS_INFO("Destination goal sent. Wait for robot to reach goal to start a new one...");
   
    // INVOKE ALL CALLBACK FUNCTIONS
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
