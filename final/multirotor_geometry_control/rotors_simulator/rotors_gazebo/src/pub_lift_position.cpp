#include <iostream>
#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <Eigen/Core>


ros::Publisher trajectory_pub1;
ros::Publisher trajectory_pub2;
ros::Publisher trajectory_pub3;
ros::Publisher trajectory_pub4;
geometry_msgs::PointStamped current_position1;
geometry_msgs::PointStamped current_position2;
geometry_msgs::PointStamped current_position3;
geometry_msgs::PointStamped current_position4;

int runs=0;
float linear_smoothing_navigation_step = 4;
bool flag1_gps_initialized_OK = false;
bool flag2_gps_initialized_OK = false;
bool flag3_gps_initialized_OK = false;
bool flag4_gps_initialized_OK = false;

double getDistance(const Eigen::Vector3d& position_err){
    double distance = 0;
    distance = position_err[0]*position_err[0] + position_err[1]*position_err[1] + position_err[2]*position_err[2];
    distance = sqrt(distance);
    return distance;
}

void move_together(const Eigen::Vector3d position){

    double uav_dist;
    uav_dist = 0.4;
    //if(runs!=1 && runs<=3){
    if(runs<=5){
        Eigen::Vector3d next_position1(position[0]-uav_dist,position[1]+uav_dist,position[2]);
        Eigen::Vector3d next_position2(position[0]+uav_dist,position[1]+uav_dist,position[2]);
        Eigen::Vector3d next_position3(position[0]-uav_dist,position[1]-uav_dist,position[2]);
        Eigen::Vector3d next_position4(position[0]+uav_dist,position[1]-uav_dist,position[2]);
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg1;
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg2;
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg3;
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg4;
        trajectory_msg1.header.stamp = ros::Time::now();
        trajectory_msg2.header.stamp = ros::Time::now();
        trajectory_msg3.header.stamp = ros::Time::now();
        trajectory_msg4.header.stamp = ros::Time::now();
        double desired_yaw = 0.0;

        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_position1,desired_yaw, &trajectory_msg1);
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_position2,desired_yaw, &trajectory_msg2);
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_position3,desired_yaw, &trajectory_msg3);
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_position4,desired_yaw, &trajectory_msg4);
        trajectory_pub1.publish(trajectory_msg1);
        trajectory_pub2.publish(trajectory_msg2);
        trajectory_pub3.publish(trajectory_msg3);
        trajectory_pub4.publish(trajectory_msg4);
        ROS_INFO("pub : [%f, %f, %f].",position[0]-uav_dist,position[1]+uav_dist,position[2]);
    }
    
    ros::Duration(1).sleep();
}

Eigen::Vector3d CenterPosition(){
    
    double px1 = current_position1.point.x;
    double py1 = current_position1.point.y;
    double pz1 = current_position1.point.z;
    double px2 = current_position2.point.x;
    double py2 = current_position2.point.y;
    double pz2 = current_position2.point.z;
    double px3 = current_position3.point.x;
    double py3 = current_position3.point.y;
    double pz3 = current_position3.point.z;
    double px4 = current_position4.point.x;
    double py4 = current_position4.point.y;
    double pz4 = current_position4.point.z;

    double p_x = (px1+px2+px3+px4)/4;
    double p_y = (py1+py2+py3+py4)/4;
    double p_z = (pz1+pz2+pz3+pz4)/4;

    Eigen::Vector3d position(p_x,p_y,p_z);
    ROS_INFO("center_position: [%f, %f, %f].",p_x,p_y,p_z);
    return position;
}

void updateUavPosition1(const geometry_msgs::PointStamped& msg)
{
    flag1_gps_initialized_OK= true;
    current_position1 = msg;
    // std::cout<<"UAV current position is: "<<msg.point.x<< msg.point.y<< msg.point.z<<std::endl;
}

void updateUavPosition2(const geometry_msgs::PointStamped& msg)
{
    flag2_gps_initialized_OK= true;
    current_position2 = msg;
}

void updateUavPosition3(const geometry_msgs::PointStamped& msg)
{
    flag3_gps_initialized_OK= true;
    current_position3 = msg;
}

void updateUavPosition4(const geometry_msgs::PointStamped& msg)
{
    flag4_gps_initialized_OK= true;
    current_position4 = msg;
}

void divideTrajectoryToTarget(Eigen::Vector3d& current_position,Eigen::Vector3d& target){

    Eigen::Vector3d position_err(target[0]-current_position[0], target[1]-current_position[1], target[2]-current_position[2]);
    Eigen::Vector3d next_position;
    double dist = getDistance(position_err);
    
    if(dist<0.1){
        //ROS_INFO("Error!!");
        return;
    }
 
    int n = int(dist)*linear_smoothing_navigation_step;
    double x_step = (target[0]-current_position[0])/n;
    double y_step = (target[1]-current_position[1])/n;
    double z_step = (target[2]-current_position[2])/n;

    if(runs<=5){
        for(int i=1; i<n+1; i++){
            next_position[0] = current_position[0] + x_step*i;
            next_position[1] = current_position[1] + y_step*i;
            next_position[2] = current_position[2] + z_step*i;
            move_together(next_position);
        }
        move_together(target);
        ROS_INFO("done!!");
    }

    runs++;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "UAV_Controler");
	ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string uav_name1 = "iris_leader1";
    std::string uav_name2 = "iris_leader2"; 
    std::string uav_name3 = "iris_follower1"; 
    std::string uav_name4 = "iris_follower2";  
    ros::param::get("~uav_name",uav_name1);
    ros::param::get("~uav_name",uav_name2);
    ros::param::get("~uav_name",uav_name3);
    ros::param::get("~uav_name",uav_name4);

    //Subscribe topic
    ros::Subscriber position_sub1 = nh.subscribe(std::string("/"+uav_name1+"/ground_truth/position").c_str(), 10, &updateUavPosition1);
    ros::Subscriber position_sub2 = nh.subscribe(std::string("/"+uav_name2+"/ground_truth/position").c_str(), 10, &updateUavPosition2);
    ros::Subscriber position_sub3 = nh.subscribe(std::string("/"+uav_name3+"/ground_truth/position").c_str(), 10, &updateUavPosition3);
    ros::Subscriber position_sub4 = nh.subscribe(std::string("/"+uav_name4+"/ground_truth/position").c_str(), 10, &updateUavPosition4);
    
    //Publish topic
    trajectory_pub1 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name1+"/command/trajectory", 10);
    trajectory_pub2 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name2+"/command/trajectory", 10);
    trajectory_pub3 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name3+"/command/trajectory", 10);
    trajectory_pub4 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name4+"/command/trajectory", 10);
	ROS_INFO("Started waypoint_publisher.");

	// Wait for some time to create the ros publisher.
	ros::Duration(2).sleep();

	double d_x= 13.05;
    double d_y= 8.98;
    double d_z= 5.20;
    
	Eigen::Vector3d point1(11.0,8.0,6.0);
    Eigen::Vector3d point2(7.0,10.0,6.0);
    Eigen::Vector3d point3(7.5,16.0,6.0);
    Eigen::Vector3d point4(7.5,19.0,6.0);
    Eigen::Vector3d point5(15.2,19.0,6.0);
	//Eigen::Vector3d point2(13.0,14.0,6.0);
	//Eigen::Vector3d point3(15.2,19.0,6.0);
    //Eigen::Vector3d point4(15.2,19.0,2.2);
    Eigen::Vector3d point6(15.2,19.0,1.7);
	Eigen::Vector3d point[6] = {point1,point2,point3,point4,point5,point6};
	//Eigen::Vector3d point4(15.2,19.0,2.3);
	while (ros::ok())
    { 
        bool flag_gps_initialized_OK = flag1_gps_initialized_OK+flag2_gps_initialized_OK+flag3_gps_initialized_OK+flag4_gps_initialized_OK;
        if(flag_gps_initialized_OK && runs<=5)
        {   
		
            Eigen::Vector3d center_position = CenterPosition();
            divideTrajectoryToTarget(center_position,point[runs]);
			//divideTrajectoryToTarget(center_position,point4);
        }
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

	ros::spinOnce();
	ros::shutdown();

	return 0;
}
