// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "marine_msgs/NavEulerStamped.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "project11/mutex_protected_bag_writer.h"
#include <regex>
#include "boost/date_time/posix_time/posix_time.hpp"

ros::Publisher position_pub;
ros::Publisher heading_pub;
ros::Publisher speed_pub;
ros::Publisher local_pos_pub;

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

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    geographic_msgs::GeoPointStamped gps;
    gps.header = msg->header;
    gps.position.latitude = msg->latitude;
    gps.position.longitude = msg->longitude;
    gps.position.altitude = msg->altitude;
    position_pub.publish(gps);
    log_bag.write("/position",ros::Time::now(),gps);
}

void headingCallback(const std_msgs::Float64::ConstPtr& msg)
{
    last_boat_heading = msg->data;
    marine_msgs::NavEulerStamped nes;
    //nes.header = msg->header;
    nes.orientation.heading = msg->data*180.0/M_PI;
    heading_pub.publish(nes);
    log_bag.write("/heading",ros::Time::now(),nes);
}

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    speed_pub.publish(msg);
    log_bag.write("/sog",ros::Time::now(),msg);
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

void sendLocalPose(const ros::TimerEvent event)
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
        mavros_msgs::CommandBoolRequest req;
        req.value = false;
        mavros_msgs::CommandBoolResponse resp;
        arm_service.call(req,resp);
        
        mavros_msgs::SetModeRequest sm_req;
        sm_req.base_mode = 4;
        sm_req.custom_mode = "GUIDED";
        mavros_msgs::SetModeResponse sm_resp;
        mode_service.call(sm_req,sm_resp);

        req.value = true;
        arm_service.call(req,resp);
    }
    else
    {
        mavros_msgs::CommandBoolRequest req;
        req.value = false;
        mavros_msgs::CommandBoolResponse resp;
        arm_service.call(req,resp);
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

    heading_pub = n.advertise<marine_msgs::NavEulerStamped>("/heading",1);
    position_pub = n.advertise<geographic_msgs::GeoPointStamped>("/position",1);
    speed_pub = n.advertise<geometry_msgs::TwistStamped>("/sog",1);
    local_pos_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_position/local", 10);

    ros::Subscriber echo_helm_sub = n.subscribe("/cmd_vel",5,twistCallback);
    ros::Subscriber active_sub = n.subscribe("/active",10,activeCallback);
    ros::Subscriber dspeed_sub = n.subscribe("/moos/desired_speed",10,desiredSpeedCallback);
    ros::Subscriber dheading_sub = n.subscribe("/moos/desired_heading",10,desiredHeadingCallback);
    ros::Subscriber position_sub = n.subscribe("/mavros/global_position/global",10,globalPositionCallback);
    ros::Subscriber speed_sub = n.subscribe("/mavros/global_position/raw/gps_vel",10,velocityCallback);
    ros::Subscriber heading_sub = n.subscribe("/mavros/global_position/compass_hdg",10,headingCallback);
    
    arm_service = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_service = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    ros::Timer timer = n.createTimer(ros::Duration(0.1),sendLocalPose);
    
    ros::spin();
    
    log_bag.close();
    
    return 0;
}
