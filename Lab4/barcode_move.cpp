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
#define INITIALIZE_VALUE -1       // VARIABLES INITIALLY HOLD VALUE OF (-1)
#define PI 3.14159265359          // RATIO OF CIRCLE, CIRCUMFERENCE:DIAMETER
#include "std_msgs/String.h"

// DEFINING GLOBAL VARIABLES 
bool flag; 				// 'FLAG' = FALSE WHEN CONDITIONS ARE MET
int sides=INITIALIZE_VALUE;             // NUMBER OF SIDES OF SHAPE ENTERED BY THE USER
double current_angle=0; 		// CURRENT ANGLE OF THE ROBOT
double target_speed=0.0; 		// ROBOT TRAVELLING SPEED (METERS/SECOND)
double target_angle=INITIALIZE_VALUE;   // DESIRED ANGLE OF ROBOT PATH (THETA)
double target_forward=INITIALIZE_VALUE; // DISTANCE THE ROBOT IS TRYING TO MOVE
double moved=0; 			// DISTANCE TRAVELLED (FORWARD, IN METERS) 
double initialx=INITIALIZE_VALUE;       // INITIAL X COORDINATE OF A MOVE
double initialy=INITIALIZE_VALUE;       // INITIAL Y COORDINATE OF A MOVE
int barcode_number;
char barcode_1[20] = "705632441947";    // BARCODE 1
char barcode_2[20] = "051111407592";    // BARCODE 2
char barcode_3[20] = "123456789012";    // BAROCDE 3

// THIS FUNCTION HANDLES FORWARD MOVEMENT OF THE ROBOT... ----------------------
// WILL CHECK TO SEE IF THE DISTANCE MOVED IS LESS THAN THE TARGET -------------
// ACCEPTS tf::tfMessage cvalue (CONSTANT) -------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void forwardprog(const tf::tfMessage cvalue)
{
  double dx, dy; // VARIABLES FOR X,Y COORDINATES

  // SETS THE INITIAL X AND Y COORDINATES
  if(initialx==INITIALIZE_VALUE || initialy==INITIALIZE_VALUE)
  {
    initialx=cvalue.transforms[0].transform.translation.x;
    initialy=cvalue.transforms[0].transform.translation.y;
  }

  // CALCULATES DISTANCE IN X AND Y TRAVELED (ONLY CARES ABOUT FORWARD MOVEMENT)
  dx = std::abs(cvalue.transforms[0].transform.translation.x-initialx);
  dy = std::abs(cvalue.transforms[0].transform.translation.y-initialy);
  
  // CALCULATES TOTAL DISTANCE THE ROBOT HAS TRAVELLED
  moved = sqrt(dx*dx+dy*dy);
  
  // CALCULATES DISTANCE (METERS) THE ROBOT HAS YET TO TRAVEL
  if(moved>target_forward)
  {
    flag=false;
  }

  // SETS A SPEED PROPORTIONAL TO THE DISTANCE YET TO BE TRAVELED
  // PLUS AN OFFSET TO ACCOUNT FOR FRICTION
  // SPEED IS IN M/S
  target_speed = std::abs(target_forward - moved)/4+0.1;
}

// THIS FUNCTION 
// ACCEPTS tf::tfMessage cvalue (CONSTANT) -------------------------------------
// RETURNS NOTHING (VOID) ------------------------------------------------------
// =============================================================================
void Turnprog(const tf::tfMessage cvalue)
{
  double turnz, turnw, mindist;

  // CALCULATE ORIENTATION OF THE ROBOT
  turnz = cvalue.transforms[0].transform.rotation.z;
  turnw = cvalue.transforms[0].transform.rotation.w;

  // CALCULATE CURRENT ANGLE OF THE ROBOT
  current_angle = 2*atan2(turnz,turnw);

  // CONVERTS THE CURRENT ANGLE TO BE BETWEEN 0 AND 2PI
  if(current_angle < 0)
  {
    current_angle =  current_angle + 2*PI;
  }
  if(current_angle >= 2*PI)
  {
    current_angle =  current_angle - 2*PI;
  }
  
  // SETS THE TARGET ANGLE
  if(target_angle == INITIALIZE_VALUE)
  {
    // IF BARCODE 1, THEN TURN CW BY 90 DEGREES
    if(barcode_number == 1)
    {  
      target_angle =  current_angle - PI/2;
    }
    // IF BARCODE 2, THEN TURN CCW BY 90 DEGREES
    else if(barcode_number == 2)
    {  
      target_angle =  current_angle + PI/2;
    }
    // IF BARCODE 3, THEN TURN CCW BY 180 DEGREES
    else if(barcode_number == 3)
    {  
      target_angle =  current_angle + PI;
    }
  }
  
  // CONVERTS THE TARGET ANGLE TO BE BETWEEN 0 AND 2PI
  if(target_angle < 0)
  {
    target_angle =  target_angle + 2*PI;
  }
  if(target_angle >= 2*PI)
  {
    target_angle =  target_angle - 2*PI;
  }

  // DETERMINES IF THE ROBOT HAS PASSED THE TARGET ANGLE 
  // THE LOGIC AFTER && ACCOUNTS FOR GOING FROM A HIGH CURRENT ANGLE,
  // SUCH AS 7PI/4, TO A LOW VALUE, SUCH AS 0
  // FOR CLOCKWISE ROTATION
  if(barcode_number == 1) 
  {
    if((current_angle<=target_angle)&&(std::abs(current_angle-target_angle)<0.1))
    {
      flag = false;
    }
  }
  // FOR COUNTERCLOCKWISE ROTATION
  else if(barcode_number == 2 || barcode_number == 3)
  {
    if((current_angle>=target_angle)&&(std::abs(current_angle-target_angle)<0.1))
    {
      flag = false;
    }
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
  target_speed = mindist/PI+0.5;
}


// MAIN PROGRAM - TAKES BARCODE SCANNED BY CAMERA AND PERFORM DIFFERENT SHAPES BASED ON BARCODE DATA
int main(int argc, char **argv)
{
  // NAMES THE PROGRAM (FOR VISUAL PURPOSES)
  ros::init(argc, argv, "lab4_barcodes");
  
  // NODEHANDLE::ADVERTISE() RETURNS A ROS::PUBLISHER OBJECT, 
  // WHICH SERVES TWO PURPOSES: 1) IT CONTAINS A PUBLISH() METHOD THAT 
  // LETS YOU PUBLISH MESSAGES ONTO THE TOPIC IT WAS CREATED WITH, AND 
  // 2) WHEN IT GOES OUT OF SCOPE, IT WILL AUTOMATICALLY UNADVERTISE.
  ros::NodeHandle n;
  // DECLARE A SECOND NODE
  ros::NodeHandle n2;
  // PUBLISHER DECLARATION FOR THE VELOCITY OF THE ROBOT
  ros::Publisher vel_pub = n2.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  
  ros::Subscriber tf; // SUBSCRIBER DECLARATION TO GET THE TRANSFORMATION MATRICES OF THE ROBOT
  
  // SETS THE FREQUENCY FOR WHICH THE PROGRAM SLEEPS AT. 1000=1/1000 SECOND
  // A ROS::RATE OBJECT ALLOWS YOU TO SPECIFY A FREQUENCY THAT YOU WOULD LIKE TO 
  // LOOP AT. IT WILL KEEP TRACK OF HOW LONG IT HAS BEEN SINCE THE LAST CALL 
  // TO RATE::SLEEP(), AND SLEEP FOR THE CORRECT AMOUNT OF TIME. Ref:
  // http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
  ros::Rate loop_rate(100);

  // SETS THE CURRENT TIME TO BE 0
  ros::Time begin = ros::Time::now();

  // INITIALIZES THE ROBOT VELOCITY
  geometry_msgs::Twist c;
  c.linear.x=0.0;
  c.linear.y=0.0;
  c.linear.z=0.0;
  c.angular.x=0.0;
  c.angular.y=0.0;
  c.angular.z=0.0;
  
  // CREATES A TIME VARIABLE
  ros::Time go;
  
  // GET SPECIFIC SCANNED BARCODE
  while(ros::ok())
  {
    // STOP ROBOT WHILE WAITING
    c.linear.x = 0.0;
    c.angular.z = 0.0;
    vel_pub.publish(c); 

    // WAIT FOR CONFIRMED BARCODE
    std_msgs::String msg =
      *ros::topic::waitForMessage<std_msgs::String>("/barcode_confirmed", n);

    if (msg.data == barcode_1)
    {
      std::cout << "Barcode 1\n";
      barcode_number = 1;
    }
    else if (msg.data == barcode_2)
    {
      std::cout << "Barcode 2\n";
      barcode_number = 2;
    }
    else if (msg.data == barcode_3)
    {
      std::cout << "Barcode 3\n";
      barcode_number = 3;
    }
    else
    {
      std::cout << "invalid barcode\n";
      continue; // WAIT FOR ANOTHER BARCODE, NO ROBOT MOVEMENT
    }
	
    // BARCODE 1 & 2: PERFORM A SQUARE (CW or CWW)
    if(barcode_number == 1 || barcode_number == 2)
    {
      target_forward = 1; // SET DISTANCE OF SIDE
      
      // PERFORM SQUARE
      for(int i=0; i<4; i++)
      {
        // FORWARD 1m
        flag = true;
        moved = 0;
        initialx = INITIALIZE_VALUE;
        initialy = INITIALIZE_VALUE;

        tf = n.subscribe("/tf", 1, forwardprog);
        ros::spinOnce();
        loop_rate.sleep();

        while(flag && ros::ok())
        {
          c.linear.x = target_speed;
          c.angular.z = 0;
          vel_pub.publish(c);

          ros::spinOnce();
          loop_rate.sleep();
        }
        tf.shutdown();

        // STOP
        go = ros::Time::now();
        while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
        {
          c.linear.x = 0;
          c.angular.z = 0;
          vel_pub.publish(c);

          ros::spinOnce();
          loop_rate.sleep();
        }

        // TURN 90 DEGREES
        flag = true;
        target_angle = INITIALIZE_VALUE;

        tf = n.subscribe("/tf", 1, Turnprog);
        ros::spinOnce();
        loop_rate.sleep();

        while(flag && ros::ok())
        {
          c.linear.x = 0;

          if(barcode_number == 1) 
          {
            c.angular.z = -target_speed; // CW
          }
          else 
          {
            c.angular.z = target_speed; // CWW
          }
          vel_pub.publish(c);

          ros::spinOnce();
          loop_rate.sleep();
        }
        tf.shutdown();
		
	// INCREMENT TARGET ANGLE
	if (barcode_number == 1)
        {
          target_angle = target_angle - PI/2;
        }
	else if (barcode_number == 2)
        {
	  target_angle = target_angle + PI/2;
        }
		 	 
        // STOP
        go = ros::Time::now();
        while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
        {
          c.linear.x = 0;
          c.angular.z = 0;
          vel_pub.publish(c);

          ros::spinOnce();
          loop_rate.sleep();
        }
      }
    }
    // BARCODE 3: TURN 180 DEGREES, FORWARD 2m, TURN 180 DEGREES, FORWARD 2m
    else if(barcode_number == 3)
    {
      // PERFORM LINE
      for(int step=0; step<2; step++)
      {
        // TURN 180 DEGREES
        flag = true;
        target_angle = INITIALIZE_VALUE;

        tf = n.subscribe("/tf", 1, Turnprog);
        ros::spinOnce();
        loop_rate.sleep();

        while(flag && ros::ok())
        {
          c.linear.x = 0;
          c.angular.z = target_speed; // CCW
          vel_pub.publish(c);

          ros::spinOnce();
          loop_rate.sleep();
        }
        tf.shutdown();
        
        // INCREMENT TARGET ANGLE
        target_angle = target_angle + PI;

        // STOP
        go = ros::Time::now();
        while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
        {
          c.linear.x = 0;
          c.angular.z = 0;
          vel_pub.publish(c);

          ros::spinOnce();
          loop_rate.sleep();
        }

        // FORWARD 2m
        target_forward = 2; // SET LENGTH OF LINE
        flag = true;
        moved = 0;
        initialx = INITIALIZE_VALUE;
        initialy = INITIALIZE_VALUE;

        tf = n.subscribe("/tf", 1, forwardprog);
        ros::spinOnce();
        loop_rate.sleep();

        while(flag && ros::ok())
        {
          c.linear.x = target_speed;
          c.angular.z = 0;
          vel_pub.publish(c);

          ros::spinOnce();
          loop_rate.sleep();
        }
        tf.shutdown();

        // STOP
        go = ros::Time::now();
        while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
        {
          c.linear.x = 0;
          c.angular.z = 0;
          vel_pub.publish(c);

          ros::spinOnce();
          loop_rate.sleep();
        }
      }
    }
    barcode_number = 0; // RESET BARCODE NUMBER FOR NEXT SCAN
  }
  return 0;
}

