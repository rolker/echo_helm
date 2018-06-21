// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "marine_msgs/NavEulerStamped.h"
#include "project11/mutex_protected_bag_writer.h"
#include <regex>
#include "boost/date_time/posix_time/posix_time.hpp"

ros::Publisher position_pub;
ros::Publisher heading_pub;
ros::Publisher speed_pub;

ros::ServiceClient arm_service;
ros::ServiceClient mode_service;
ros::ServiceClient setpoint_position_service;

double heading;
double rudder;
double throttle;
ros::Time last_time;

double last_boat_heading;

double desired_speed;
ros::Time desired_speed_time;
double desired_heading;
ros::Time desired_heading_time;


MutexProtectedBagWriter log_bag;


void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    throttle = msg->twist.linear.x;
    rudder = -msg->twist.angular.z;
    
    last_time = msg->header.stamp;
}

void desiredSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& inmsg)
{
    desired_speed = inmsg->twist.linear.x;
    desired_speed_time = inmsg->header.stamp;
}

void desiredHeadingCallback(const marine_msgs::NavEulerStamped::ConstPtr& inmsg)
{
    desired_heading = inmsg->orientation.heading;
    desired_heading_time = inmsg->header.stamp;
}

void sendHeadingHold(const ros::TimerEvent event)
{
    bool doDesired = true;
    if (!last_time.isZero())
    {
        if(event.last_real-last_time>ros::Duration(.5))
        {
            throttle = 0.0;
            rudder = 0.0;
        }
        else
            doDesired = false;
    }

    ros::Duration delta_t = event.current_real-event.last_real;
    heading = last_boat_heading + rudder; //*delta_t.toSec();
    heading = fmod(heading,M_PI*2.0);
    if(heading < 0.0)
        heading += M_PI*2.0;
    
    if(doDesired)
    {
    }
    else
    {
    }
    //asvMsg.thrust.type = asv_msgs::Thrust::THRUST_SPEED;
    if(doDesired)
    {
        if (event.current_real - desired_heading_time < ros::Duration(.5) && event.current_real - desired_speed_time < ros::Duration(.5))
        {
        }
    }
    //log_bag.write("/control/drive/heading_hold",ros::Time::now(),asvMsg);
}

void activeCallback(const std_msgs::Bool::ConstPtr& inmsg)
{
    if(inmsg->data)
    {
	ros::ServiceClient arm_service = n.serviceClient<
    }
    else
    {
    }
}


int main(int argc, char **argv)
{
    heading = 0.0;
    throttle = 0.0;
    rudder = 0.0;
    last_boat_heading = 0.0;
    
    ros::init(argc, argv, "echo_helm");
    ros::NodeHandle n;

    boost::posix_time::ptime now = ros::WallTime::now().toBoost();
    std::string iso_now = std::regex_replace(boost::posix_time::to_iso_extended_string(now),std::regex(":"),"-");
    std::string log_filename = "nodes/echo_helm-"+iso_now+".bag";
    log_bag.open(log_filename, rosbag::bagmode::Write);

    

    ros::Subscriber asv_helm_sub = n.subscribe("/cmd_vel",5,twistCallback);
    ros::Subscriber activesub = n.subscribe("/active",10,activeCallback);
    ros::Subscriber dspeed_sub = n.subscribe("/moos/desired_speed",10,desiredSpeedCallback);
    ros::Subscriber dheading_sub = n.subscribe("/moos/desired_heading",10,desiredHeadingCallback);
    
    ros::Timer timer = n.createTimer(ros::Duration(0.1),sendHeadingHold);
    
    ros::spin();
    
    log_bag.close();
    
    return 0;
}
