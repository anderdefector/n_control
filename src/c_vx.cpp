#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
//Variables
//ARDrone m=1.477
//Bebop m=0.450
//Bebop2 m=0.500
float  m = 0.45, g = 9.81;
float x, dx, wx, xd = 0.0, xg = 0.0, dxg = 0.0, wxg = 0.0, x0=0.0,  dx0 = 0.0, wx0 = 0.0, xg0 = 0.0, dxg0 = 0.0, wxg0 = 0.0, ux=0.0, vx=0.0;

using namespace std;
//Funciones
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
	x=msg->pose.pose.position.x;
}

int main (int argc, char **argv){
	ros::init(argc, argv, "c_vx");
	ros::NodeHandle node_obj;
	//Publisher
	ros::Publisher vx_publisher=node_obj.advertise<std_msgs::Float32>("/velx",10);
	//Suscriber
	ros::Subscriber odom = node_obj.subscribe("/bebop/odom",10,odom_callback);
	ros::Rate loop_rate(10);
	std::cout<<"Velocidad x \n";
	FILE * PlotData = fopen("velx.txt", "w");
	while (ros::ok()){		
		vx=1;
		std_msgs::Float32 msg;
		msg.data=vx;
		vx_publisher.publish(msg);
		ros::spinOnce();
	}	
	return 0;	
}
