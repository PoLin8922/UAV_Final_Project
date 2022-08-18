#include <iostream>
#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
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
int count_x = 0;
int count_y = 0;
int total_waypoints = 22;
float max_step = 0.35;
//float linear_smoothing_navigation_step = 2;
bool flag1_gps_initialized_OK = false;
bool flag2_gps_initialized_OK = false;
bool flag3_gps_initialized_OK = false;
bool flag4_gps_initialized_OK = false;
bool tag1_gps_initialized_OK = false;
bool tag2_gps_initialized_OK = false;
bool flag_get_waypoints_x_OK = false;
bool flag_get_waypoints_y_OK = false;
bool detection = false;
Eigen::Vector3d point[26];
Eigen::Vector3d position_err1;
Eigen::Vector3d position_err2;

Eigen::Vector3d apriltag_tf(Eigen::Vector3d apriltag_err){
    float rx = 180*M_PI/180;
    float ry = 0;
    float rz = 90*M_PI/180;

    // ++ToDo++ need consider if the orientation of quadcopter change
    Eigen::Matrix3d R1, R2;
    R1 <<       1,         0,        0,
	            0,   cos(rx),  sin(rx),
	            0,  -sin(rx),  cos(rx);
    R2 <<   cos(rz),  sin(rz),         0,
	       -sin(rz),  cos(rz),         0,
	              0,        0,         1;

    Eigen::Vector3d position_err;
    position_err = R2*R1*apriltag_err;

    return position_err;
}

double getDistance(const Eigen::Vector3d& position_err){
    double distance = 0;
    distance = position_err[0]*position_err[0] + position_err[1]*position_err[1] + position_err[2]*position_err[2];
    distance = sqrt(distance);
    return distance;
}

void move_together(const Eigen::Vector3d position){

    double uav_dist;
    uav_dist = 0.4;
    if(runs<=total_waypoints+1){
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

Eigen::Vector3d TagPosition(){
     
    if(runs==total_waypoints) detection = true;
    else detection = false;
    
    ros::Duration(3).sleep();
    double px1,py1,pz1,px2,py2,pz2;
    do{
        ros::spinOnce();
    }while(position_err1[0]==0 && runs==total_waypoints);
    px1 = current_position1.point.x+(position_err1[0]);
    py1 = current_position1.point.y+(position_err1[1]);
    pz1 = current_position1.point.z+(position_err1[2]);
    px2 = current_position2.point.x+(position_err2[0]);
    py2 = current_position2.point.y+(position_err2[1]);
    pz2 = current_position2.point.z+(position_err2[2]);
    detection = false;
    ROS_INFO("position_err1 : [%f, %f, %f].",position_err1[0],position_err1[1],position_err1[2]);

    double x_offset = 0.2;
    double y_offset = -0.25;
    double z_offset = 3;
    double d_x = (px1+px2)/2+x_offset;
    double d_y = (py1+py2)/2+y_offset;
    double d_z = (pz1+pz2)/2+z_offset;

    Eigen::Vector3d position(d_x,d_y,d_z);
    ROS_INFO("-----------------------------");
    ROS_INFO("desire_position: [%f, %f, %f].",d_x,d_y,d_z);
    ROS_INFO("-----------------------------");
    
    return position;
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
    ROS_INFO("-----------------------------");
    ROS_INFO("center_position: [%f, %f, %f].",p_x,p_y,p_z);
    ROS_INFO("-----------------------------");
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

void updateTagPosition1(const apriltag_ros::AprilTagDetectionArray& msg){
    tag1_gps_initialized_OK = true;

    Eigen::Vector3d apriltag_err(0,0,0); 
    if(detection){
        apriltag_err[0] = msg.detections[0].pose.pose.pose.position.x;
        apriltag_err[1] = msg.detections[0].pose.pose.pose.position.y;
        apriltag_err[2] = msg.detections[0].pose.pose.pose.position.z;
    }

    /***** convert to world frame *****/
    position_err1 = apriltag_tf(apriltag_err);
}

void updateTagPosition2(const apriltag_ros::AprilTagDetectionArray& msg){
    tag2_gps_initialized_OK = true;

    Eigen::Vector3d apriltag_err(0,0,0);
    if(detection){
        apriltag_err[0] = msg.detections[0].pose.pose.pose.position.x;
        apriltag_err[1] = msg.detections[0].pose.pose.pose.position.y;
        apriltag_err[2] = msg.detections[0].pose.pose.pose.position.z;
    }

    /***** convert to world frame *****/
    position_err2 = apriltag_tf(apriltag_err);
}

void get_waypoint_x(const std_msgs::Float64& msg){
    
    ROS_INFO("count_x: [%d].",count_x);
    if(count_x==total_waypoints-1) flag_get_waypoints_x_OK = true;
    point[count_x][0] = msg.data;
    point[count_x][2] = 6.0;
    count_x++;
}

void get_waypoint_y(const std_msgs::Float64& msg){

    if(count_y==total_waypoints-1) flag_get_waypoints_y_OK = true;
    point[count_y][1] = msg.data;
    point[count_y][2] = 6.0;
    count_y++;
}

void divideTrajectoryToTarget(Eigen::Vector3d& current_position,Eigen::Vector3d& target){

    Eigen::Vector3d position_err(target[0]-current_position[0], target[1]-current_position[1], target[2]-current_position[2]);
    Eigen::Vector3d next_position;
    double dist = getDistance(position_err);
    
    if(dist<0.1){
        //ROS_INFO("Error!!");
        return;
    }
    
    //if(runs>total_waypoints-8) linear_smoothing_navigation_step = 4;
    //if(runs>total_waypoints-4) linear_smoothing_navigation_step = 6;
    int n = int(dist);
    //int n;
    //if(int(dist)!=0) n = int(dist)*int(dist);
    //if(runs>total_waypoints-3) linear_smoothing_navigation_step = 4;
    //n = n*linear_smoothing_navigation_step;
    double x_step = (target[0]-current_position[0])/n;
    double y_step = (target[1]-current_position[1])/n;
    double z_step = (target[2]-current_position[2])/n;
    
    //if(runs>0) max_step = 0.3;
    //if(runs>8) max_step = 0.4;
    //if(runs>total_waypoints-5) max_step = 0.2;
    if(runs>total_waypoints-2) max_step = 0.1;
    while(abs(x_step)>max_step || abs(y_step)>max_step || abs(z_step)>max_step){
        n++;
        x_step = (target[0]-current_position[0])/n;
        y_step = (target[1]-current_position[1])/n;
        z_step = (target[2]-current_position[2])/n;
    }

    if(runs==total_waypoints+1){
        ROS_INFO("-----------------------------");
        ROS_INFO("z_step: [%f] max_step: [%f].",z_step,max_step);
        ROS_INFO("-----------------------------");
    }

    if(runs<=total_waypoints+1){
        for(int i=1; i<n+1; i++){
            next_position[0] = current_position[0] + x_step*i;
            next_position[1] = current_position[1] + y_step*i;
            next_position[2] = current_position[2] + z_step*i;
            move_together(next_position);
            ros::Duration(0.2).sleep();
        }
        //move_together(target);
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
    ros::Subscriber sub1 = nh.subscribe("/tag_detections/"+uav_name2+"",1000,updateTagPosition1);
    ros::Subscriber sub2 = nh.subscribe("/tag_detections/"+uav_name4+"",1000,updateTagPosition2);
    ros::Subscriber position_sub1 = nh.subscribe(std::string("/"+uav_name1+"/ground_truth/position").c_str(), 10, &updateUavPosition1);
    ros::Subscriber position_sub2 = nh.subscribe(std::string("/"+uav_name2+"/ground_truth/position").c_str(), 10, &updateUavPosition2);
    ros::Subscriber position_sub3 = nh.subscribe(std::string("/"+uav_name3+"/ground_truth/position").c_str(), 10, &updateUavPosition3);
    ros::Subscriber position_sub4 = nh.subscribe(std::string("/"+uav_name4+"/ground_truth/position").c_str(), 10, &updateUavPosition4);
    ros::Subscriber waypoint_sub1 = nh.subscribe("waypoints_x", 10, &get_waypoint_x);
    ros::Subscriber waypoint_sub2 = nh.subscribe("waypoints_y", 10, &get_waypoint_y);
    
    //Publish topic
    trajectory_pub1 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name1+"/command/trajectory", 10);
    trajectory_pub2 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name2+"/command/trajectory", 10);
    trajectory_pub3 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name3+"/command/trajectory", 10);
    trajectory_pub4 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name4+"/command/trajectory", 10);
	ROS_INFO("Started waypoint_publisher.");

	// Wait for some time to create the ros publisher.
	ros::Duration(2).sleep();
    
    Eigen::Vector3d point1(15.2,19.0,6.0);
    Eigen::Vector3d point2(15.2,19.0,1.7);
	while (ros::ok())
    { 
        bool flag_gps_initialized_OK = flag1_gps_initialized_OK+flag2_gps_initialized_OK+flag3_gps_initialized_OK+flag4_gps_initialized_OK;
        bool flag_get_waypoints_OK = flag_get_waypoints_x_OK + flag_get_waypoints_x_OK;
        if(flag_gps_initialized_OK && flag_get_waypoints_OK)
        {  
            Eigen::Vector3d center_position = CenterPosition();
            if(runs<total_waypoints){
                ROS_INFO("waypoints [%d]: [%f, %f, %f].",runs+1,point[total_waypoints-runs-1][0],point[total_waypoints-runs-1][1],point[total_waypoints-runs-1][2]);
                //move_together(point[total_waypoints-runs-1]);
                //runs++;
                divideTrajectoryToTarget(center_position,point[total_waypoints-runs-1]);
            }
            else if(runs==total_waypoints){
                //Eigen::Vector3d center_position = CenterPosition();
                //point1 = TagPosition();
                //point1[2] = 6.0;
                divideTrajectoryToTarget(center_position,point1);
            } 
            else if(runs==total_waypoints+1){
                //Eigen::Vector3d center_position = CenterPosition();
                //point2 = point1;
                //point2[2] = 1.7;
                divideTrajectoryToTarget(center_position,point2);
            } 
        }

        ros::spinOnce();
        ros::Duration(1).sleep();
    }

	ros::spinOnce();
	ros::shutdown();

	return 0;
}
