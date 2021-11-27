/******************************************************************************
BSD 3-Clause License
Copyright (c) 2021, Mayank Joshi
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/**
 * @file main.cpp
 * @author Mayank Joshi
 * @brief 
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <turtlebot_roomba/ObstacleAvoider.hpp>

// velocity topic name
extern std::string vel_topic = "/cmd_vel";

// laser scanner topic name
extern std::string scan_topic =  "/scan";

ObstacleAvoider::ObstacleAvoider(ros::NodeHandle node_handle,
const double dist) :
node_handle_(node_handle),
collision_distance_(dist) {
// initialize velocity publisher for the robot
vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>(vel_topic, 10);

// initialize laser scanner subscriber for the robot
scan_pub_ = node_handle_.subscribe(scan_topic, 10,
&ObstacleAvoider::scan_callback, this);

ros::spinOnce();
}

ObstacleAvoider::~ObstacleAvoider() {
}

void ObstacleAvoider::scan_callback(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

    // sweep scan angle starts from 0 to 45, 0 degrees is in forward
    int start_index = 45;

    // sweep scan angle ends at 315, i.e from 315 to 360 or 0 degrees
    int end_index = 315;

    // set variable to store the minimun index of the scan angles
    int min_index_angle = -1;

    // ros geometry msg to store the velocity commands
    geometry_msgs::Twist vel_cmd;

    // laser scanner max range that can be observed, refer to scanner datasheet
    double scan_max = scan_msg->range_max;

    // variable to store the minimum distance to the obstacle
    // initialized with large value
    double min_dist_to_obstacle = scan_max;

    // iterate through the laser scan msg ranges
    for ( int i = 0; i < scan_msg->ranges.size() ; i++ ) {
        // process the scan only iff angle is between the specified region
        if (i <= start_index || i >= end_index) {
            // process the scan msg only if the ith index range is non-zero
            if (!std::isnan(scan_msg->ranges[i])) {
                // variable to store the distance at the current scan angle
                double scan_dist = scan_msg->ranges[i];

                // check if scan dist is less than min distance to an obstacle
                if (scan_dist < min_dist_to_obstacle) {
                    ROS_INFO_STREAM("OBSTACLE IN MY WAY BUT FARAWAY!!!!...");
                    // update min dist to obst to scan dist for next iteration
                    min_dist_to_obstacle = scan_dist;

                    // update min dist index angle to current angle
                    min_index_angle = i;
                }
            }
        }
    }
    // check if min dist to obstacle is less than the collision distance
    if (min_dist_to_obstacle <= collision_distance_) {
        ROS_WARN_STREAM("OBSTACLE VERY NEAR TO ME!!!! Now Turning...");
        // stop the robot
        vel_cmd.linear.x = 0.0;

        // boolean to store the direction in which robot has to rotate
        // once it encountered an obstacle
        bool rotate_clockwise = true;

        // check where does the object lie wrt the robot
        if ( min_index_angle <= 45 )  {
            // since object lies to the left side, rotate anticlockwise
            rotate_clockwise = false;
        } else if ( min_index_angle >= 315 ) {
            // since the object lies to the right side, rotate  clockwise
            rotate_clockwise = true;
        }
        // rotate the robot depending upon the direction of rotation
        vel_cmd.angular.z = rotate_clockwise ? 0.5 : -0.5;
    } else {
        ROS_INFO_STREAM("NO OBSTACLE IN MY WAY!!!! Moving Forward...");
        // move forward
        vel_cmd.linear.x = 0.3;
        vel_cmd.angular.z = 0.0;
    }

    // publish the calculated velocity commands
    vel_pub_.publish(vel_cmd);
}
