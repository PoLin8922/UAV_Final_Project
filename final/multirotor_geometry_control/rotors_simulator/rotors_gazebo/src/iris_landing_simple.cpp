#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <Eigen/Core>


ros::Publisher trajectory_pub;
geometry_msgs::PointStamped current_position;

float linear_smoothing_navigation_step = 2;
double desired_yaw = 0.0;
double husky_z_offset = 1.35;
double box_z_offset = 2.6;
bool flag_gps_initialized_OK = false;
bool tag_gps_initialized_OK = false;
Eigen::Vector3d home;
Eigen::Vector3d apriltag_err;
Eigen::Vector3d position_err;

void updateUavPosition(const geometry_msgs::PointStamped& msg)
{
    if (!flag_gps_initialized_OK)
    {
        flag_gps_initialized_OK= true;
        home[0] = msg.point.x;
        home[1] = msg.point.y;
        home[2] = msg.point.z;  //紀錄起始位子
    }
    current_position = msg;

    // std::cout<<"UAV current position is: "<<msg.point.x<< msg.point.y<< msg.point.z<<std::endl;
}
  

void updateTagPosition(const apriltag_ros::AprilTagDetectionArray& msg){
    tag_gps_initialized_OK = true;
    apriltag_err[0] = msg.detections[0].pose.pose.pose.position.x;
    apriltag_err[1] = msg.detections[0].pose.pose.pose.position.y;
    apriltag_err[2] = msg.detections[0].pose.pose.pose.position.z;

    /***** convert to world frame *****/
    Eigen::Matrix3d R1, R2;
    // initial: x +180 degree -> z -90 degree
    float rx = 180*M_PI/180;
    float ry = 0;
    float rz = 90*M_PI/180;

    // ++ToDo++ need consider if the orientation of quadcopter change
    
    R1 <<       1,         0,        0,
	            0,   cos(rx),  sin(rx),
	            0,  -sin(rx),  cos(rx);
    R2 <<   cos(rz),  sin(rz),         0,
	       -sin(rz),  cos(rz),         0,
	              0,        0,         1;
    position_err = R2*R1*apriltag_err;

    /*ROS_INFO("position error relate to camera: [%f, %f, %f].",
	    position_err[0],
	    position_err[1],
	    position_err[2]);*/
}

double getDistance(const Eigen::Vector3d& position_err){
    double distance = 0;
    distance = position_err[0]*position_err[0] + position_err[1]*position_err[1] + position_err[2]*position_err[2];
    distance = sqrt(distance);
    return distance;
}


void divideTrajectoryToTarget(const Eigen::Vector3d& target){
    Eigen::Vector3d position_err(target[0]-current_position.point.x, target[1]-current_position.point.y, target[2]-current_position.point.z);
    Eigen::Vector3d next_position;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    double dist = getDistance(position_err);
    int n = int(dist)*linear_smoothing_navigation_step;
    double x_step = (target[0]-current_position.point.x)/n;
    double y_step = (target[1]-current_position.point.y)/n;
    double z_step = (target[2]-current_position.point.z)/n;

    for(int i=1; i<n+1; i++){
        next_position[0] = current_position.point.x + x_step*i;
        next_position[1] = current_position.point.y + y_step*i;
        next_position[2] = current_position.point.z + z_step*i;

	    trajectory_msg.header.stamp = ros::Time::now();
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_position,
	                desired_yaw, &trajectory_msg);
	    trajectory_pub.publish(trajectory_msg);
        ros::Duration(1).sleep();
    }
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(target,
	                desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    ROS_INFO("done!!");
}


int main(int argc, char **argv){
	ros::init(argc, argv, "UAV_Controler");
	ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");

    std::string uav_name = "iris_leader1";  
    ros::param::get("~mav_name",uav_name);

    //Subscribe topic
	ros::Subscriber sub = nh.subscribe("/tag_detections/"+uav_name+"",1000,updateTagPosition);
    ros::Subscriber position_sub = nh.subscribe(std::string("/"+uav_name+"/ground_truth/position").c_str(), 10, &updateUavPosition);
    
    //Publish topic
    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name+"/command/trajectory", 10);


    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();

    
    while (ros::ok())
    { 
        ros::Duration(3).sleep();
        if(flag_gps_initialized_OK && tag_gps_initialized_OK)
        {   

            double d_x = current_position.point.x+(position_err[0]);  //totation matrix
            double d_y = current_position.point.y+(position_err[1]);
            double d_z = current_position.point.z+(position_err[2])+box_z_offset;
            Eigen::Vector3d desired_position(d_x,d_y,d_z);
            ROS_INFO("current_position : [%f, %f, %f].",current_position.point.x,current_position.point.y,current_position.point.z);
            ROS_INFO("desire_position: [%f, %f, %f].",d_x,d_y,d_z);
            ROS_INFO("error : [%f, %f, %f].",position_err[0],position_err[1],position_err[2]);
            ROS_INFO("getDistanceToTarget(position_err) : %f", getDistance(position_err));
            
            if(getDistance(position_err)>0.1){
                ROS_INFO("pub desire position: [%f, %f, %f].",d_x,d_y,d_z);
                divideTrajectoryToTarget(desired_position);
            }
            
        }
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    
	ros::shutdown();
	return 0;
}