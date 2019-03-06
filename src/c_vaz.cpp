#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
//Variables
//ARDrone m=1.477
//Bebop m=0.450
//Bebop2 m=0.500
float  m = 0.45, g = 9.81;
float psi, dpsi, wpsi, psid = 0.0, psig = 0.0, dpsig = 0.0, wpsig = 0.0, psi0=0.0, dpsi0 = 0.0, wpsi0 = 0.0, psig0 = 0.0, dpsig0 = 0.0, wpsig0 = 0.0, upsi=0.0, vaz=0.0;
using namespace std;
//Funciones
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
	psi=msg->pose.pose.orientation.z;
}

int main (int argc, char **argv){
	ros::init(argc, argv, "c_vaz");
	ros::NodeHandle node_obj;
	//Publisher
	ros::Publisher vaz_publisher=node_obj.advertise<std_msgs::Float32>("/velaz",10);
	//Suscriber
	ros::Subscriber odom = node_obj.subscribe("/bebop/odom",10,odom_callback);
	ros::Rate loop_rate(10);
	std::cout<<"Velocidad Az \n";
	FILE * PlotData = fopen("velaz.txt", "w");
	while (ros::ok()){
		
		vaz=1;
		std_msgs::Float32 msg;
		msg.data=vaz;
		vaz_publisher.publish(msg);
		ros::spinOnce();
	}	
	return 0;	
}
