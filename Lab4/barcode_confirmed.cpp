// NAMES: Jordi Fraile Raton, Valentina Parodi
// CLASS: EGRS372
// PROJECT: Lab 4
// PURPOSE: Read barcodes in real-time from a camera through ROS and utilize visual information for a mobile robot application

// INCLUDE LIBRARIES
#include "ros/ros.h"              // INCLUDE ROS (ROBOT OPERATING SYSTEM) 
#include <sstream>                // LIBRARY THAT PROVIDES STRING CLASSES
#include <iostream> 		  // FOR INPUTS AND OUTPUTS FOR CIN (INPUT)
#include "math.h"   		  // FOR ABSOLUTE VALUE AND TRIG FUNCTIONS
#include "geometry_msgs/Twist.h"  // PUBLISHING A GEOETRY_MSGS/...
#include "tf/tfMessage.h"  	  // SUBSCRIBING TO TF/TFMESSAGE ...
#include <string.h>
#include "std_msgs/String.h"

// DEFINE GLOBAL VARIABLES
int count=0;
std_msgs::String barcode;
bool flag = true; 

// Scan Function reads barcode and increment counts if identical to previously scanned barcode
void scan_barcode(const std_msgs::String string_barcode)
{
  // If barcode matches previous one
  if (string_barcode.data == barcode.data)
  {
    count++; // Increment count
  }
  else
  {
    // If new barcode detected, reset counter
    barcode.data = string_barcode.data;
    count = 1; // Reset counter to 1
  }
  // If scanned 5 times
  if (count >= 5)
  {
    std::cout << "Sending Barcode" << std::endl;
    flag = false;
    count = 0; // Reset counter
  }
  std::cout << "Identical number: " << count << std::endl;
}

// Main Program: subscribes to the topic "barcode" and outputs the data barcode
int main(int argc, char **argv)
{
  // Declare string for barcode published
  std_msgs::String pub_barcode;
  // Names the program for visual purposes
  ros::init(argc, argv, "barcode_confirmed");
  // Initialize nodes
  ros::NodeHandle n_in;
  ros::NodeHandle n_out;
  // Sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(1000);

  // Declare subscriber "barcode" is the name of the node
  // 1000 is how many to save in the buffer
  // scan_barcode is the function called when a value is received
  ros::Subscriber input = n_in.subscribe("barcode", 1000, scan_barcode);
  
  // Declare publisher "barcode" is the name of the node
  // 1000 is the number of values to keep stored until they are overwritten
  ros::Publisher output = n_out.advertise<std_msgs::String>("barcode_confirmed", 1000);

  // rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
    // Process incoming barcode messages
    ros::spinOnce();

    // If Scanner reaches 5 matches in a row, publish the confirmed barcode
    if (!flag)
    {
      pub_barcode.data = barcode.data; // Publish the confirmed barcode
      output.publish(pub_barcode);
      flag = true; // Reset for next barcode scan
    }
    loop_rate.sleep();
  }
  return 0;
}

