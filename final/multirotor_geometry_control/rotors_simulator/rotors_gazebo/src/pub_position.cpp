/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_publisher");
	ros::NodeHandle nh;
	ros::Publisher trajectory_pub_leader1 =
	        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
	               "/iris_leader1/command/trajectory", 10);
	ros::Publisher trajectory_pub_leader2 =
	        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
	               "/iris_leader2/command/trajectory", 10);
	ros::Publisher trajectory_pub_follower1 =
	        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
	               "/iris_follower1/command/trajectory", 10);
	ros::Publisher trajectory_pub_follower2 =
	        nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
	               "/iris_follower2/command/trajectory", 10);

	ROS_INFO("Started waypoint_publisher.");

    double d_x= 13.05;
    double d_y= 8.98;
    double d_z= 1.35;
    double yaw= 0.0;
	double delay= 1.0;
	const float DEG_2_RAD = M_PI / 180.0;

	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_leader1;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_leader2;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_follower1;
	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg_follower2;
	trajectory_msg_leader1.header.stamp = ros::Time::now();
	trajectory_msg_leader2.header.stamp = ros::Time::now();
	trajectory_msg_follower1.header.stamp = ros::Time::now();
	trajectory_msg_follower2.header.stamp = ros::Time::now();

	Eigen::Vector3d desired_position_leader1(d_x-0.4,d_y+0.4,d_z);
	Eigen::Vector3d desired_position_leader2(d_x+0.4,d_y+0.4,d_z);
	Eigen::Vector3d desired_position_follower1(d_x-0.4,d_y-0.4,d_z);
	Eigen::Vector3d desired_position_follower2(d_x+0.4,d_y-0.4,d_z);

	double desired_yaw = yaw*DEG_2_RAD;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position_leader1,
	                desired_yaw, &trajectory_msg_leader1);
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position_leader2,
	                desired_yaw, &trajectory_msg_leader2);
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position_follower1,
	                desired_yaw, &trajectory_msg_follower1);
	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position_follower2,
	                desired_yaw, &trajectory_msg_follower2);

	// Wait for some time to create the ros publisher.
	ros::Duration(delay).sleep();
   
	while (trajectory_pub_leader1.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}
	while (trajectory_pub_leader2.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}
	while (trajectory_pub_follower1.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}
	while (trajectory_pub_follower2.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}

	ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
	         nh.getNamespace().c_str(),
	         d_x,
	         d_y,
	         d_z);

	trajectory_pub_leader1.publish(trajectory_msg_leader1);
	trajectory_pub_leader2.publish(trajectory_msg_leader2);
	trajectory_pub_follower1.publish(trajectory_msg_follower1);
	trajectory_pub_follower2.publish(trajectory_msg_follower2);

	ros::spinOnce();
	ros::shutdown();

	return 0;
}
