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
bool flag_gps_initialized_OK = false;
bool tag_gps_initialized_OK = false;
Eigen::Vector3d home;
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
    position_err[0] = msg.detections[0].pose.pose.pose.position.x;
    position_err[1] = msg.detections[0].pose.pose.pose.position.y;
    position_err[2] = msg.detections[0].pose.pose.pose.position.z;

    /*ROS_INFO("position_err : [%f, %f, %f].",
	    position_err[0],
	    position_err[1],
	    position_err[2]);*/
}

double getDistance(const Eigen::Vector3d& err)
{
    double temp = 0;
    temp = err[0]*err[0] + err[1]*err[1] + err[2]*err[2];
    temp = sqrt(temp);
    return temp;
}

double getDistanceToTarget(const Eigen::Vector3d& target)
{
    double temp = 0;
    temp += (target[0] - current_position.point.x)*(target[0] - current_position.point.x);
    temp += (target[1] - current_position.point.y)*(target[1] - current_position.point.y);
    temp += (target[2] - current_position.point.z)*(target[2] - current_position.point.z);
    temp = sqrt(temp);
    return temp;
}

bool reachTargetPosition(const Eigen::Vector3d& target, float max_error)
{
    double temp = getDistanceToTarget(target);

    if (temp < max_error)
        return true;
    return false;
}

bool linearSmoothingNavigationTask(const Eigen::Vector3d& target)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();


    double dist = getDistanceToTarget(target);
    if (reachTargetPosition(target,0.1))
        return true;

    ROS_INFO("dist:%f",dist);
    Eigen::Vector3d next_step;

    if(dist<linear_smoothing_navigation_step)
    {
        next_step = target;
    }
    else
    {
        next_step[0] = current_position.point.x+(target[0]-current_position.point.x)/dist*linear_smoothing_navigation_step;
        next_step[1] = current_position.point.y+(target[1]-current_position.point.y)/dist*linear_smoothing_navigation_step;
        next_step[2] = current_position.point.z+(target[2]-current_position.point.z)/dist*linear_smoothing_navigation_step;
    }

    double desired_yaw = 0.0; 

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_step, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);

    /*ROS_INFO("Publishing waypoint on : [%f, %f, %f].",
	         next_step[0],
	         next_step[1],
	         next_step[2]);*/

    return false;
}


int main(int argc, char **argv){
	ros::init(argc, argv, "UAV_Controler");
	ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");

    std::string uav_name = "iris_leader";  
    ros::param::get("~mav_name",uav_name);

    //Subscribe topic
	ros::Subscriber sub = nh.subscribe("/tag_detections",1000,updateTagPosition);
    ros::Subscriber position_sub = nh.subscribe(std::string("/"+uav_name+"/odometry_sensor1/position").c_str(), 10, &updateUavPosition);
    
    //Publish topic
    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name+"/command/trajectory", 10);

    
    while (ros::ok())
    { 
        
        if(flag_gps_initialized_OK && tag_gps_initialized_OK)
        {   
            ros::Duration(2).sleep();
            double z_offset = 1.5;
            double d_x = current_position.point.x-(position_err[1]);  //totation matrix
            double d_y = current_position.point.y-(position_err[0]);
            double d_z = current_position.point.z-(position_err[2])+z_offset;
            double desired_yaw = 0.0;

            ROS_INFO("current_position : [%f, %f, %f].",current_position.point.x,current_position.point.y,current_position.point.z);
            ROS_INFO("error : [%f, %f, %f].",position_err[0],position_err[1],position_err[2]);
            ROS_INFO("desire_position: [%f, %f, %f].",d_x,d_y,d_z);
            Eigen::Vector3d desired_position(d_x,d_y,d_z);
            
            ROS_INFO("getDistanceToTarget(position_err) : %f", getDistance(position_err));
            if(getDistance(position_err)>0.01){
                ROS_INFO("pub_desire_position: [%f, %f, %f].",d_x,d_y,d_z);
                bool temp = linearSmoothingNavigationTask(desired_position);    
            }
            
        }

        ros::spinOnce();
    }

	ros::shutdown();
	return 0;
}