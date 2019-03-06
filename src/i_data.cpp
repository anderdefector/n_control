#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <termios.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>
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
//Funciones Callback
/*void battery_callback(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr& msg2){
//	battery=msg2->percent;
//}
/Callback nodos velocidad*/
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
  ros::init(argc, argv, "i_data");
  ros::NodeHandle nodo_;
  //teclado	
  //ros::Subscriber tec_sub = nodo_.subscribe("/number1",10,number_callback1);
  //ros::Subscriber battery_ = nodo_.subscribe("/bebop/states/common/CommonState/BatteryStateChanged",1,battery_callback);
  //Nodos de velocidad
  ros::Subscriber vx_sub = nodo_.subscribe("/velx",10,vx_callback);  
  ros::Subscriber vy_sub = nodo_.subscribe("/vely",10,vy_callback);
  ros::Subscriber vz_sub = nodo_.subscribe("/velz",10,vz_callback);
  ros::Subscriber vaz_sub = nodo_.subscribe("/velaz",10,vaz_callback);								
   //Hover	
  std::cout<<"Master Control Node \n";

  while(ros::ok()){
	std::cout<<"Contol Automatico"<<"Vx= " <<vx<<" Vy= " <<vy<<" Vz= "<<vz<<" Vaz= "<<vaz<<"\n";      	
	ros::spinOnce();
	//ros::spin();
  }
  return 0;
}
