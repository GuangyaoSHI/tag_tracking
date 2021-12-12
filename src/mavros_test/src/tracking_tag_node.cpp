/**
 * @file mavros_test_node.cpp
 *
 * Copyright (c) 2020 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * For a license to use on non-ModalAI hardware, please contact
 * license@modalai.com
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define FLIGHT_ALTITUDE 1.5f
#define RATE            50  // loop rate hz
#define RADIUS          5.0 // radius of figure 8 in meters
#define CYCLE_S         8   // time to complete one figure 8 cycle in seconds
#define STEPS           (CYCLE_S*RATE)

#define PI  3.14159265358979323846264338327950


mavros_msgs::State current_state;
//mavros_msgs::PositionTarget path[STEPS];


//geometry_msgs::PoseStamped tag_pose_body
std_msgs::Float64 tag_pose_body_x; 
std_msgs::Float64 tag_pose_body_y; 
std_msgs::Float64 tag_pose_body_z; 

//mavros_msgs::PositionTarget vel_cmd;
//vel_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
//vel_cmd.type_mask =  vel_cmd.IGNORE_AFX | vel_cmd.IGNORE_AFY | vel_cmd.IGNORE_AFZ | vel_cmd.FORCE | vel_cmd.IGNORE_YAW_RATE | /////////vel_cmd.IGNORE_PX | vel_cmd.IGNORE_PY | vel_cmd.IGNORE_PZ;

// generate a path following Bernoulli's lemiscate as a parametric equation
// note this is in ENU coordinates since mavros will convert to NED
// x right, y forward, z up.


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


void tag_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{ 
    geometry_msgs::PoseStamped  tag_pose = *msg;
    tag_pose_body_x.data = tag_pose.pose.position.z + 0.1;
    tag_pose_body_y.data = -tag_pose.pose.position.x;
    tag_pose_body_z.data = -(tag_pose.pose.position.y + 0.4);
    //the orientation is ignored for the first step
    //tag_pose_body.pose.orientation = tag_pose.pose.orientation;
}


int main(int argc, char **argv)
{
    int i;

    ros::init(argc, argv, "tracking_tag_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub           = nh.subscribe<mavros_msgs::State>
                                        ("mavros/state", 10, state_cb);
    ros::Subscriber tag_pose_sub        = nh.subscribe<geometry_msgs::PoseStamped>
					("/tag_detections/tagpose", 10, tag_pose_cb);
    //ros::Publisher local_pos_pub        = nh.advertise<geometry_msgs::PoseStamped>
    //                                    ("mavros/setpoint_position/local", 10);
    ros::Publisher body_pos_pub        = nh.advertise<geometry_msgs::PoseStamped>
                                        ("/tag_pose_body", 10);
    ros::Publisher body_rel_x_pub        = nh.advertise<std_msgs::Float64>
                                        ("/x/state", 10);
    ros::Publisher body_rel_y_pub        = nh.advertise<std_msgs::Float64>
                                        ("/y/state", 10);
    ros::Publisher body_rel_z_pub        = nh.advertise<std_msgs::Float64>
                                        ("/z/state", 10);
    ros::ServiceClient arming_client    = nh.serviceClient<mavros_msgs::CommandBool>
                                        ("mavros/cmd/arming");
    ros::ServiceClient land_client      = nh.serviceClient<mavros_msgs::CommandTOL>
                                        ("mavros/cmd/land");
    ros::ServiceClient set_mode_client  = nh.serviceClient<mavros_msgs::SetMode>
                                        ("mavros/set_mode");
    ros::Publisher target_local_pub     = nh.advertise<mavros_msgs::PositionTarget>
                                        ("mavros/setpoint_raw/local", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(RATE);
    

    /*{

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCT...");
    }

    // keep this pose constant, home position
    mavros_msgs::PositionTarget position_home;
    position_home.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_home.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    position_home.position.x = 0;
    position_home.position.y = 0;
    position_home.position.z = FLIGHT_ALTITUDE;
    position_home.velocity.x = 0;
    position_home.velocity.y = 0;
    position_home.velocity.z = 0;
    position_home.acceleration_or_force.x = 0;
    position_home.acceleration_or_force.y = 0;
    position_home.acceleration_or_force.z = 0;
    // path starts pointing 45 degrees right of forward (y axis in ENU)
    // ENU yaw is angle left (CCW) of X axis which is to the right.
    // hence yaw here is -45 degrees
    // plus 90 gets us from x axis as 0 to y axis as 0
    position_home.yaw = (0.0f + 90.0f) * PI / 180.0f;
    position_home.yaw_rate = 0;

    //send a few setpoints before starting
    for(i = 100; ros::ok() && i > 0; --i){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }


HOME:
    ROS_INFO("waiting for offboard mode");
    // wait for the system to be armed and in offboard mode
    while(ros::ok()){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
        if(current_state.mode == "OFFBOARD" && current_state.armed) break;
    }

    // give the system 2 seconds to get to home position
    i = RATE * 2;
    ROS_INFO("going home");
    while(ros::ok() && i>0){
        // return to home position if px4 falls out of offboard mode or disarms
        if(current_state.mode != "OFFBOARD" || !current_state.armed){
            goto HOME;
        }
        i--;
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

}*/
    // now begin publish relative position of the tag wrt body frame

    ROS_INFO("start to publish tag pose wrt body frame");
    while(ros::ok()){
        // return to home position if px4 falls out of offboard mode or disarms
        //if(current_state.mode != "OFFBOARD" || !current_state.armed){
        //    goto HOME;
        //}
        /*{
        body_pos_pub.publish(tag_pose_body);
        }*/
        body_rel_x_pub.publish(tag_pose_body_x);
        body_rel_y_pub.publish(tag_pose_body_y);
        body_rel_z_pub.publish(tag_pose_body_z);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


/* HOW TO ARM , LAND AND CHANGE MODE

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    if( set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
    }

    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }

        ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
*/
