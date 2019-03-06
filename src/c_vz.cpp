#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <stdio.h>
//Variables
//ARDrone m=1.477
//Bebop m=0.450
//Bebop2 m=0.500
float h=0.0, hdes=1.0;
float m = 1.477, g = 9.81,e0 = 0.0,e,dz,u,a0 = 0, th=0.0, phi=0.0;
float zg=0.0,dzg=0.0,ddzg=0.0,w=0.0,wg=0.0,zg0=0.0,dzg0=0.0,ddzg0=0.0,w0=0.0,wg0=0.0,vz=0.0;

using namespace std;
//Funciones
void altitude_callback(const sensor_msgs::Range::ConstPtr& msg2){
	h=msg2->range;
}

int main (int argc, char **argv){
	ros::init(argc, argv, "c_vz");
	ros::NodeHandle node_obj;
	//Publisher
	ros::Publisher vz_publisher=node_obj.advertise<std_msgs::Float32>("/velz",10);
	//Suscriber
	ros::Subscriber Alt = node_obj.subscribe("/sonar_height",1,altitude_callback);
	ros::Rate loop_rate(10);
	std::cout<<"Velocidad Z \n";
	while (ros::ok()){
		//Inicio Control de Altura		
		e = hdes - h;
		dz = h-a0;	
		u = ((1.0*(e)-dz)/(cos(th)*cos(phi)));		
		cout<<"z = "<<h<<" e = "<<e<<" u = "<<u<<"\n";	
		if(u>1){	
			u=1;
		}
		if(u<-1){
			u=-1;
		}
		//Observador
 		zg = zg0 + 0.001*dzg0 + pow(0.0001,2)*u/(2.0*m) + tanh(a0-zg0);
		dzg = dzg0 + 0.0001*u/m + tanh(a0-zg0);
		w = a0-zg0;
		wg = wg0-w + tanh(a0-zg0-wg0);
		//Fin de control de Altura
		cout << "VelZ" << vz << "\n";
		vz=u;
		std_msgs::Float32 msg;
		msg.data=vz;
		vz_publisher.publish(msg);
		//Reinicio de variables
		a0 = h;	zg0 = zg; dzg0 = dzg; wg = wg0;
		ros::spinOnce();
	}	
	return 0;	
}
