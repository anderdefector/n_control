#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
//Variables
//ARDrone m=1.477
//Bebop m=0.450
//Bebop2 m=0.500
float  m = 1.477, g = 9.81;
float y, dy, wy, yd = 0.0, yg = 0.0, dyg = 0.0, wyg = 0.0, y00=0.0, dy0 = 0.0, wy0 = 0.0, yg0 = 0.0, dyg0 = 0.0, wyg0 = 0.0, uy=0.0, vy=0.0;

using namespace std;
//Funciones
void odom_callback(const tf2_msgs::TFMessage::ConstPtr& msg){
	y=msg->transforms[0].transform.translation.y;	
}

int main (int argc, char **argv){
	ros::init(argc, argv, "c_vy");
	ros::NodeHandle node_obj;
	//Publisher
	ros::Publisher vy_publisher=node_obj.advertise<std_msgs::Float32>("/vely",10);
	//Suscriber
	ros::Subscriber odom = node_obj.subscribe("/tf",10,odom_callback);
	ros::Rate loop_rate(10);
	std::cout<<"Velocidad y \n";
	//FILE * PlotData = fopen("vely.txt", "w");
	while (ros::ok()){
		//Inicio de control de velocidad en Y
		uy = 0.8*(yd-y)-0.2*(y-y00)-0.1*wyg;
		if(uy>0.7)
		{	
			uy=0.7;
		}
		if(uy<-0.7)
		{
			uy=-0.7;
		}
		yg = yg0 + 0.001*dyg0 + pow(0.001,2)*uy/(2.0*m) + tanh(y00-yg0);
		dyg = dyg0 + 0.001*uy/m + tanh(y00-yg0);
		wy = y00-yg0;
		wyg = wyg0 + tanh(wy0-wyg0);
		//Fin de control de velocidad en Y
		cout << " Y = "<< y << " e= "<< yd-y << " VelY" << vy << "\n";
		vy=uy;
		std_msgs::Float32 msg;
		msg.data=vy;
		vy_publisher.publish(msg);
		//Reinicio de variables
		y00 = y; yg0 = yg; wy0 = wy; wyg0 = wyg;
		ros::spinOnce();
	}	
	return 0;	
}
