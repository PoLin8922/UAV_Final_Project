#include <iostream>
#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <Eigen/Core>

using namespace std;


ros::Publisher trajectory_pub1;
ros::Publisher trajectory_pub2;
ros::Publisher trajectory_pub3;
ros::Publisher trajectory_pub4;
geometry_msgs::PointStamped current_position1;
geometry_msgs::PointStamped current_position2;
geometry_msgs::PointStamped current_position3;
geometry_msgs::PointStamped current_position4;

int runs=1;
float linear_smoothing_navigation_step = 4;
bool flag1_gps_initialized_OK = false;
bool flag2_gps_initialized_OK = false;
bool flag3_gps_initialized_OK = false;
bool flag4_gps_initialized_OK = false;
bool tag1_gps_initialized_OK = false;
bool tag2_gps_initialized_OK = false;
bool tag3_gps_initialized_OK = false;
bool tag4_gps_initialized_OK = false;
bool detection = false;
Eigen::Vector3d position_err1;
Eigen::Vector3d position_err2;
Eigen::Vector3d position_err3;
Eigen::Vector3d position_err4;


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

/*
void move_to_center(const Eigen::Vector3d position){
    
    detection = false;
    double uav_dist=0.4;
    Eigen::Vector3d err1(position_err1[0]-uav_dist,position_err1[1]+uav_dist,0);
    Eigen::Vector3d err2(position_err2[0]+uav_dist,position_err2[1]+uav_dist,0);
    Eigen::Vector3d err3(position_err3[0]-uav_dist,position_err3[1]-uav_dist,0);
    Eigen::Vector3d err4(position_err4[0]+uav_dist,position_err4[1]-uav_dist,0);
    ROS_INFO("err1 : [%f, %f, %f].",err1[0],err1[1],err1[2]);
    ROS_INFO("err2 : [%f, %f, %f].",err2[0],err2[1],err2[2]);
    ROS_INFO("err3 : [%f, %f, %f].",err3[0],err3[1],err3[2]);
    ROS_INFO("err4 : [%f, %f, %f].",err4[0],err4[1],err4[2]);
    double dist1 = getDistance(err1);
    double dist2 = getDistance(err2);
    double dist3 = getDistance(err3);
    double dist4 = getDistance(err4);

    int num[4] = {int(dist1), int(dist2), int(dist3),int(dist4)};
    int max = num[0];
    for(int i=0; i<4; i++){
        if(num[i]>max) max=num[i];
    }
    
    int step = max*linear_smoothing_navigation_step;
    double x1_step = (position_err1[0]-uav_dist)/step;
    double y1_step = (position_err1[1]+uav_dist)/step;
    double x2_step = (position_err2[0]+uav_dist)/step;
    double y2_step = (position_err2[1]+uav_dist)/step;
    double x3_step = (position_err3[0]-uav_dist)/step;
    double y3_step = (position_err3[1]-uav_dist)/step;
    double x4_step = (position_err4[0]+uav_dist)/step;
    double y4_step = (position_err4[1]-uav_dist)/step;
    ROS_INFO("err1 : [%f, %f].",x1_step,y1_step);
    ROS_INFO("err2 : [%f, %f].",x2_step,y2_step);
    ROS_INFO("err3 : [%f, %f].",x3_step,y3_step);
    ROS_INFO("err4 : [%f, %f].",x4_step,y4_step);

    for(int i=3; i<step+1; i++){
        Eigen::Vector3d next_position1(position[0]-x1_step*(step-i)-uav_dist,position[1]-y1_step*(step-i)+uav_dist,position[2]);
        Eigen::Vector3d next_position2(position[0]-x2_step*(step-i)+uav_dist,position[1]-y2_step*(step-i)+uav_dist,position[2]);
        Eigen::Vector3d next_position3(position[0]-x3_step*(step-i)-uav_dist,position[1]-y3_step*(step-i)-uav_dist,position[2]);
        Eigen::Vector3d next_position4(position[0]-x4_step*(step-i)+uav_dist,position[1]-y4_step*(step-i)-uav_dist,position[2]);
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
        ROS_INFO("pub1 : [%f, %f, %f].",position[0]-x1_step*(step-i),position[1]-y1_step*(step-i),position[2]);
        ros::Duration(1).sleep();
    }
    ros::Duration(1).sleep();
    detection = true;
    
}*/

void move_together(const Eigen::Vector3d position){

    double uav_dist;
    if(runs==1) uav_dist = 0.5;
    if(runs==2) uav_dist = 0.5;
    if(runs==3) uav_dist = 0.4;
    //if(runs!=1 && runs<=3){
    if(runs<=3){
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
        //trajectory_pub1.publish(trajectory_msg1);
        //trajectory_pub2.publish(trajectory_msg2);
        //trajectory_pub3.publish(trajectory_msg3);
        //trajectory_pub4.publish(trajectory_msg4);
        ROS_INFO("pub2 : [%f, %f, %f].",position[0]-uav_dist,position[1]+uav_dist,position[2]);
    }
    
    ros::Duration(1).sleep();
}

void divideTrajectoryToTarget(const Eigen::Vector3d& current_position, const Eigen::Vector3d& target){
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
    
    //if(runs==1) move_to_center(target);
    //else{
    //detection = false;
    if(runs<=3){
        for(int i=1; i<n+1; i++){
            next_position[0] = current_position[0] + x_step*i;
            next_position[1] = current_position[1] + y_step*i;
            next_position[2] = current_position[2] + z_step*i;
            move_together(next_position);
        }
        move_together(target);
        ROS_INFO("done!!");
    }
    
    //ros::Duration(15).sleep();
    //detection = true;

    //if(runs==3) detection = false;
    runs++;
}

Eigen::Vector3d TagPosition(){

    detection = true;
    ros::Duration(3).sleep();
    bool flag_gps_initialized_OK = flag1_gps_initialized_OK+flag2_gps_initialized_OK+flag3_gps_initialized_OK+flag4_gps_initialized_OK;
    double px1,py1,pz1,px2,py2,pz2,px3,py3,pz3,px4,py4,pz4;
    if(flag_gps_initialized_OK){
        px1 = current_position1.point.x+(position_err1[0]);
        py1 = current_position1.point.y+(position_err1[1]);
        pz1 = current_position1.point.z+(position_err1[2]);
        px2 = current_position2.point.x+(position_err2[0]);
        py2 = current_position2.point.y+(position_err2[1]);
        pz2 = current_position2.point.z+(position_err2[2]);
        px3 = current_position3.point.x+(position_err3[0]);
        py3 = current_position3.point.y+(position_err3[1]);
        pz3 = current_position3.point.z+(position_err3[2]);
        px4 = current_position4.point.x+(position_err4[0]);
        py4 = current_position4.point.y+(position_err4[1]);
        pz4 = current_position4.point.z+(position_err4[2]);
        flag1_gps_initialized_OK = false;
        flag2_gps_initialized_OK = false;
        flag3_gps_initialized_OK = false;
        flag4_gps_initialized_OK = false;
        detection = false;
    }

    double z_offset;
    if(runs==1) z_offset=8.5;
    if(runs==2) z_offset=7.6;
    if(runs==3) z_offset=2.6;

    double d_x = (px1+px2+px3+px4)/4;
    double d_y = (py1+py2+py3+py4)/4;
    double d_z = (pz1+pz2+pz3+pz4)/4+z_offset;

    Eigen::Vector3d position(d_x,d_y,d_z);
    //std::cout<<position_err1[2];
    ROS_INFO("desire_position: [%f, %f, %f].",d_x,d_y,d_z);
    
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
    //ROS_INFO("position error relate to camera: [%f, %f, %f].",
	    //position_err1[0],
	    //position_err1[1],
	    //position_err1[2]);
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

void updateTagPosition3(const apriltag_ros::AprilTagDetectionArray& msg){
    tag3_gps_initialized_OK = true;

    Eigen::Vector3d apriltag_err(0,0,0);
    if(detection){
        apriltag_err[0] = msg.detections[0].pose.pose.pose.position.x;
        apriltag_err[1] = msg.detections[0].pose.pose.pose.position.y;
        apriltag_err[2] = msg.detections[0].pose.pose.pose.position.z;
    }

    /***** convert to world frame *****/
    position_err3 = apriltag_tf(apriltag_err);
}

void updateTagPosition4(const apriltag_ros::AprilTagDetectionArray& msg){
    tag4_gps_initialized_OK = true;

    Eigen::Vector3d apriltag_err(0,0,0);
    if(detection){
        apriltag_err[0] = msg.detections[0].pose.pose.pose.position.x;
        apriltag_err[1] = msg.detections[0].pose.pose.pose.position.y;
        apriltag_err[2] = msg.detections[0].pose.pose.pose.position.z;
    }

    /***** convert to world frame *****/
    position_err4 = apriltag_tf(apriltag_err);
}


int main(int argc, char **argv){
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
	ros::Subscriber sub1 = nh.subscribe("/tag_detections/"+uav_name1+"",1000,updateTagPosition1);
    ros::Subscriber sub2 = nh.subscribe("/tag_detections/"+uav_name2+"",1000,updateTagPosition2);
    ros::Subscriber sub3 = nh.subscribe("/tag_detections/"+uav_name3+"",1000,updateTagPosition3);
    ros::Subscriber sub4 = nh.subscribe("/tag_detections/"+uav_name4+"",1000,updateTagPosition4);
    ros::Subscriber position_sub1 = nh.subscribe(std::string("/"+uav_name1+"/ground_truth/position").c_str(), 10, &updateUavPosition1);
    ros::Subscriber position_sub2 = nh.subscribe(std::string("/"+uav_name2+"/ground_truth/position").c_str(), 10, &updateUavPosition2);
    ros::Subscriber position_sub3 = nh.subscribe(std::string("/"+uav_name3+"/ground_truth/position").c_str(), 10, &updateUavPosition3);
    ros::Subscriber position_sub4 = nh.subscribe(std::string("/"+uav_name4+"/ground_truth/position").c_str(), 10, &updateUavPosition4);
    
    //Publish topic
    trajectory_pub1 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name1+"/command/trajectory", 10);
    trajectory_pub2 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name2+"/command/trajectory", 10);
    trajectory_pub3 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name3+"/command/trajectory", 10);
    trajectory_pub4 = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/"+uav_name4+"/command/trajectory", 10);

    while (ros::ok())
    { 
        bool flag_gps_initialized_OK = flag1_gps_initialized_OK+flag2_gps_initialized_OK+flag3_gps_initialized_OK+flag4_gps_initialized_OK;
        bool tag_gps_initialized_OK = tag1_gps_initialized_OK+tag2_gps_initialized_OK+tag3_gps_initialized_OK+tag4_gps_initialized_OK;
        if(flag_gps_initialized_OK && tag_gps_initialized_OK)
        {   
            Eigen::Vector3d center_position = CenterPosition();
            Eigen::Vector3d desire_position = TagPosition();
            divideTrajectoryToTarget(center_position,desire_position);
            /*
            ROS_INFO("current_position : [%f, %f, %f].",current_position.point.x,current_position.point.y,current_position.point.z);
            ROS_INFO("desire_position: [%f, %f, %f].",d_x,d_y,d_z);
            ROS_INFO("error : [%f, %f, %f].",position_err[0],position_err[1],position_err[2]);
            ROS_INFO("getDistanceToTarget(position_err) : %f", getDistance(position_err));
            
            if(getDistance(position_err)>0.1){
                ROS_INFO("pub desire position: [%f, %f, %f].",d_x,d_y,d_z);
                divideTrajectoryToTarget(desired_position);
            }*/
            
        }
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    
	ros::shutdown();
	return 0;
}