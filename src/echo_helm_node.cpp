// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "marine_msgs/NavEulerStamped.h"
#include "marine_msgs/Heartbeat.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/State.h>
#include "project11/gz4d_geo.h"
#include <regex>
#include "boost/date_time/posix_time/posix_time.hpp"

ros::Publisher position_pub;
ros::Publisher heading_pub;
ros::Publisher magnetic_heading_pub;
ros::Publisher speed_pub;
ros::Publisher local_pos_pub;
ros::Publisher heartbeat_pub;

ros::ServiceClient arm_service;
ros::ServiceClient mode_service;
ros::ServiceClient frame_service;

double heading;
double rudder;
double throttle;
ros::Time last_time;
double magnetic_declination;

double last_boat_heading;

double desired_speed;
ros::Time desired_speed_time;
double desired_heading;
ros::Time desired_heading_time;

std::string piloting_mode;

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    throttle = msg->twist.linear.x;
    rudder = -msg->twist.angular.z;
    
    last_time = msg->header.stamp;
    
    //std::cerr << "last_time: " << last_time << std::endl;
    //std::cerr << "throttle: " << throttle << " rudder: " << rudder << std::endl;
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    geographic_msgs::GeoPointStamped gps;
    gps.header = msg->header;
    gps.position.latitude = msg->latitude;
    gps.position.longitude = msg->longitude;
    gps.position.altitude = msg->altitude;
    position_pub.publish(gps);
}

void headingCallback(const std_msgs::Float64::ConstPtr& msg)
{
    last_boat_heading = msg->data+magnetic_declination;
    marine_msgs::NavEulerStamped nes;
    nes.header.stamp = ros::Time::now();
    //nes.header = msg->header;
    nes.orientation.heading = msg->data+magnetic_declination;//*180.0/M_PI;
    heading_pub.publish(nes);
    nes.orientation.heading = msg->data;
    magnetic_heading_pub.publish(nes);
}

void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    speed_pub.publish(msg);
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

void magneticDeclinationCallback(const std_msgs::Float32::ConstPtr& inmsg)
{
    magnetic_declination = inmsg->data;
}

void sendLocalPose(const ros::TimerEvent event)
{
    geometry_msgs::TwistStamped ts;
    bool doDesired = true;
    if (!last_time.isZero())
    {
        //std::cerr << "last time: " << last_time << "  event time: " << event.last_real << std::endl;
        if(event.last_real-last_time>ros::Duration(.5))
        {
            throttle = 0.0;
            rudder = 0.0;
        }
        else
        {
            //std::cerr << "cmd_vel timeout" << std::endl;
            doDesired = false;
        }
    }

    ros::Duration delta_t = event.current_real-event.last_real;
    //heading = last_boat_heading + rudder; //*delta_t.toSec();
    //heading = fmod(heading,M_PI*2.0);
    //if(heading < 0.0)
        //heading += M_PI*2.0;

    if(doDesired)
    {
        if (event.current_real - desired_heading_time < ros::Duration(.5) && event.current_real - desired_speed_time < ros::Duration(.5))
        {
            ts.header.stamp = desired_heading_time;
            ts.header.frame_id = "base_link";
            ts.twist.linear.x = desired_speed;
            float delta_heading = desired_heading - last_boat_heading;
            while (delta_heading > 180.0)
                delta_heading -= 360.0;
            while (delta_heading < -180.0)
                delta_heading += 360.0;
            //std::cerr << "delta_heading: " << delta_heading << std::endl;
            ts.twist.angular.z = -gz4d::Radians(delta_heading);
        }
        else
        {
            //std::cerr << "desired times out of range" << std::endl;
        }
    }
    else
    {
        ts.header.stamp = last_time;
        ts.header.frame_id = "base_link";
        ts.twist.linear.x = throttle;
        ts.twist.angular.z = -rudder;
    }
    //std::cerr << "ts.twist.linear.x: " << ts.twist.linear.x << "  angular.z: " << ts.twist.angular.z << std::endl; 
    local_pos_pub.publish(ts);
}

void helmModeCallback(const std_msgs::String::ConstPtr& inmsg)
{
    if(piloting_mode == "standby" && inmsg->data != "standby")
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
        
        mavros_msgs::SetMavFrameRequest smf_req;
        smf_req.mav_frame = 8;
        mavros_msgs::SetMavFrameResponse smf_resp;
        frame_service.call(smf_req,smf_resp);
    }
    if(inmsg->data == "standby")
    {
        mavros_msgs::CommandBoolRequest req;
        req.value = false;
        mavros_msgs::CommandBoolResponse resp;
        arm_service.call(req,resp);
    }
    piloting_mode = inmsg->data;
}

std::string boolToString(bool value)
{
    if(value)
        return "true";
    return "false";
}

void stateCallback(const mavros_msgs::State::ConstPtr& inmsg)
{
    marine_msgs::Heartbeat hb;
    hb.header = inmsg->header;

    marine_msgs::KeyValue kv;

    kv.key = "piloting_mode";
    kv.value = piloting_mode;
    hb.values.push_back(kv);
    
    kv.key = "connected";
    kv.value = boolToString(inmsg->connected);
    hb.values.push_back(kv);

    kv.key = "armed";
    kv.value = boolToString(inmsg->armed);
    hb.values.push_back(kv);

    kv.key = "guided";
    kv.value = boolToString(inmsg->guided);
    hb.values.push_back(kv);

    kv.key = "mode";
    kv.value = inmsg->mode;
    hb.values.push_back(kv);
    
    heartbeat_pub.publish(hb);
}

int main(int argc, char **argv)
{
    heading = 0.0;
    throttle = 0.0;
    rudder = 0.0;
    last_boat_heading = 0.0;
    magnetic_declination = 0.0;
    piloting_mode = "standby";
    
    ros::init(argc, argv, "echo_helm");
    ros::NodeHandle n;

    heading_pub = n.advertise<marine_msgs::NavEulerStamped>("/heading",1);
    magnetic_heading_pub = n.advertise<marine_msgs::NavEulerStamped>("/magnetic_heading",1);
    position_pub = n.advertise<geographic_msgs::GeoPointStamped>("/position",1);
    speed_pub = n.advertise<geometry_msgs::TwistStamped>("/sog",1);
    local_pos_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    heartbeat_pub = n.advertise<marine_msgs::Heartbeat>("/heartbeat", 10);

    ros::Subscriber echo_helm_sub = n.subscribe("/cmd_vel",5,twistCallback);
    ros::Subscriber dspeed_sub = n.subscribe("/project11/desired_speed",10,desiredSpeedCallback);
    ros::Subscriber dheading_sub = n.subscribe("/project11/desired_heading",10,desiredHeadingCallback);
    ros::Subscriber position_sub = n.subscribe("/mavros/global_position/raw/fix",10,globalPositionCallback);
    ros::Subscriber speed_sub = n.subscribe("/mavros/global_position/raw/gps_vel",10,velocityCallback);
    ros::Subscriber heading_sub = n.subscribe("/mavros/global_position/compass_hdg",10,headingCallback);
    ros::Subscriber magnetic_declination_sub = n.subscribe("/magnetic_declination",10,magneticDeclinationCallback);
    ros::Subscriber helmmodesub = n.subscribe("/project11/piloting_mode",10,helmModeCallback);
    ros::Subscriber state_sub = n.subscribe("/mavros/state",10,stateCallback);

    
    arm_service = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_service = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    frame_service = n.serviceClient<mavros_msgs::SetMavFrame>("mavros/setpoint_velocity/mav_frame");
    
    ros::Timer timer = n.createTimer(ros::Duration(0.1),sendLocalPose);
    
    ros::spin();

    return 0;
}
