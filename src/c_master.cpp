#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <termios.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
//Variables
int intOp;
uint8_t battery;
float vx=0.0,vy=0.0,vz=0.0,vaz=0.0;
geometry_msgs::Twist emma_cmd;

//Funciones
void c_vel (float lin_x, float lin_y, float lin_z, float ang_z);
//Funciones Callback
void opt_callback(const std_msgs::Int32::ConstPtr& msg1){
	intOp=msg1->data;
}
/*void battery_callback(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr& msg2){
	battery=msg2->percent;
}*/
//Callback nodos velocidad
void vx_callback(const std_msgs::Float32::ConstPtr& msg3){
	vx=msg3->data;
}
void vy_callback(const std_msgs::Float32::ConstPtr& msg4){
	vy=msg4->data;
}
void vz_callback(const std_msgs::Float32::ConstPtr& msg5){
	vz=msg5->data;
}
void vaz_callback(const std_msgs::Float32::ConstPtr& msg6){
	vaz=msg6->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MCN");
  ros::NodeHandle nodo_;
  ros::Publisher takeoff_pub_ = nodo_.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  ros::Publisher land_pub_ = nodo_.advertise<std_msgs::Empty>("/ardrone/land", 1);
  ros::Publisher fb_pub = nodo_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  //teclado	
  ros::Subscriber tec_sub = nodo_.subscribe("/copt",10,opt_callback);
  //ros::Subscriber battery_ = nodo_.subscribe("/bebop/states/common/CommonState/BatteryStateChanged",1,battery_callback);
  //Nodos de velocidad
  ros::Subscriber vx_sub = nodo_.subscribe("/velx",1,vx_callback);  
  ros::Subscriber vy_sub = nodo_.subscribe("/vely",1,vy_callback);
  ros::Subscriber vz_sub = nodo_.subscribe("/velz",1,vz_callback);
  ros::Subscriber vaz_sub = nodo_.subscribe("/velaz",1,vaz_callback);								
  	
  std_msgs::Empty takeoff_cmd;
  std_msgs::Empty land_cmd;
  std_msgs::Empty emergency;	
  //Hover	
  std::cout<<"Master Control Node \n";

  while(ros::ok()){
	switch(intOp){
		case 49:
			std::cout<<"C.A. "<<" Vx= " <<vx<<" Vy= " <<vy<<" Vz= "<<vz<<" Vaz= "<<vaz<<"\n";
			c_vel(0.5,vy,vz,0);
			fb_pub.publish(emma_cmd);
		break;
		case 50:
			std::cout<<"Take Off \n";
			takeoff_pub_.publish(takeoff_cmd);
		break;
		case 51:
			std::cout<<"Land \n";
			land_pub_.publish(land_cmd);
		break;
		case 52:
			std::cout<<"Hover \n";
			c_vel(0.0,0.0,0.0,0.0);
			fb_pub.publish(emma_cmd);
		break;
		default:	
			std::cout<<"Hover \n";
			c_vel(0.0,0.0,0.0,0.0);
			fb_pub.publish(emma_cmd);
		break;
	}      	
	ros::spinOnce();
	//ros::spin();
  }
  return 0;	
}

void c_vel (float lin_x, float lin_y, float lin_z, float ang_z){
	emma_cmd.linear.x = lin_x;
  	emma_cmd.linear.y = lin_y;
  	emma_cmd.linear.z = lin_z;
  	emma_cmd.angular.z = ang_z;
	
}
	
