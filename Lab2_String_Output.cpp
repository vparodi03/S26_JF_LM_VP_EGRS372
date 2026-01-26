// NAMES: Jordi Fraile Raton, Leo Mandoka, Valentina Parodi
// CLASS: EGRS372
// PROJECT: Lab 2 - String Output program
// PURPOSE: Develop ROS program in C++ and learn control communication via publishing and subscribing

//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include "math.h"
#include <iostream>
#include <string.h>
#include "std_msgs/String.h"

//output function: publishes the output message
void out_function(const std_msgs::String String_Output)
{
  //output the data
  //string
  std::cout << "The output string: " << String_Output.data.c_str() << " was published" <<std::endl;
  //string length
  std::cout << "The length of the output string is: " << strlen(String_Output.data.c_str()) <<std::endl;
}

//main program: subscribes to the topic "string_topic" and outputs both the string and the length of the string
int main(int argc, char **argv)
{
  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_String_Output");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare subscriber "string_topic" is the name of the node
  //1000 is how many to save in the buffer
  //out_function is the function called when a value is received
  ros::Subscriber output = n.subscribe("string_topic", 1000, out_function);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
    //looks for data
    ros::spin();
  }

  return 0;
}
