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

#ifndef INCLUDE_TURTLEBOT_ROOMBA_OBSTACLEAVOIDER_HPP_
#define INCLUDE_TURTLEBOT_ROOMBA_OBSTACLEAVOIDER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <vector>


class ObstacleAvoider {
 public:
    /**
     * @brief ObstacleAvoider class constructor
     * 
     * @param ros node handle
     */
    explicit ObstacleAvoider(ros::NodeHandle node_handle,
    const double dist = 0.45);

    /**
     * @brief ObstacleAvoider class destructor
     * 
     */
    ~ObstacleAvoider();


 private:
    /**
     * @brief Laser Scan callback method
     * instead of scanning 360 degrees, we span only 90 degrees region 
     * i.e we scan only the region sweeping an area from  
     * 0-45 degrees and 315-0 degrees
     * 
     * and based on that navigate forward or rotate
     * d
     * @param scan_data 
     * 
     * @return Void
     * 
     */
    void scan_callback(
        const sensor_msgs::LaserScan::ConstPtr& scan_msg);


 private:
    /**
     * @brief ros node handle for ObstacleAvoider class
     * 
     */

    ros::NodeHandle node_handle_;

    /**
     * @brief distance to avoid collision
     * 
     */
    double collision_distance_;  // meteres

    /**
     * @brief velocity Publisher
     * 
     */
    ros::Publisher vel_pub_;

    /**
     * @brief laser scan Subscriber
     * 
     */
    ros::Subscriber scan_pub_;
};

#endif  // INCLUDE_TURTLEBOT_ROOMBA_OBSTACLEAVOIDER_HPP_
