// NAMES: Jordi Fraile Raton, Leo Mandoka, Valentina Parodi
// CLASS: EGRS372
// PROJECT: Lab 2 - String Input program
// PURPOSE: Develop ROS program in C++ and learn control communication via publishing and subscribing

//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include "math.h"
#include <iostream>
#include <string.h>
#include "std_msgs/String.h"

//main program Input: takes a string from the user and publish it to a topic called "string_topic"
int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_String_Input");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare variables
  std_msgs::String String_Input;
  std_msgs::String String_pub;

  //declare publisher "string_topic" is the name of the node
  //1000 is the number of values to keep stored until they are overwritten
  ros::Publisher input = n.advertise<std_msgs::String>("string_topic", 1000);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
    //clear the input buffer
    std::fflush;

    //prompt the user for an input
    std::cout << "Enter an input string: ";
    //get the input from the user
    std::getline(std::cin, String_Input.data);

    //confirm the string is being sent
    std::cout << "Sending the output string: " << String_Input.data << std::endl;

    //set the message
    String_pub.data = String_Input.data;
    //publish the data
    input.publish(String_pub);
    
    //sends out any data necessary then waits based on the loop rate
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

