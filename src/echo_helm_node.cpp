// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/TwistStamped.h"
#include "project11_msgs/Heartbeat.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/State.h>

ros::Publisher status_pub;
ros::Publisher cmd_vel_pub;

ros::ServiceClient arm_service;
ros::ServiceClient mode_service;
ros::ServiceClient frame_service;

bool standby;

void cmdVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  if(!standby)
    cmd_vel_pub.publish(msg);
}

void standbyCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if(standby && !msg->data)
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
  standby = msg->data;
  if(msg->data)
  {
    mavros_msgs::CommandBoolRequest req;
    req.value = false;
    mavros_msgs::CommandBoolResponse resp;
    arm_service.call(req,resp);
  }
}

std::string boolToString(bool value)
{
    if(value)
        return "true";
    return "false";
}

void stateCallback(const mavros_msgs::State::ConstPtr& inmsg)
{
    project11_msgs::Heartbeat hb;
    hb.header = inmsg->header;

    project11_msgs::KeyValue kv;

    kv.key = "project11_standby";
    kv.value = boolToString(standby);
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
    
    status_pub.publish(hb);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "echo_helm");
    ros::NodeHandle n;

    status_pub = n.advertise<project11_msgs::Heartbeat>("project11/status/helm", 10);
    
    cmd_vel_pub = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber cmd_vel_sub = n.subscribe("project11/control/cmd_vel", 10, cmdVelCallback);

    standby = true;
    ros::Subscriber stanby_sub = n.subscribe("project11/piloting_mode/standby/active", 5, standbyCallback);
    
    ros::Subscriber state_sub = n.subscribe("mavros/state",10,stateCallback);

    arm_service = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_service = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    frame_service = n.serviceClient<mavros_msgs::SetMavFrame>("mavros/setpoint_velocity/mav_frame");
    
    ros::spin();

    return 0;
}
