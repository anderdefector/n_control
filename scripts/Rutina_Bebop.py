#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
#Librerias para pose ardrone en simulador
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Range

x =0 
y_odom = 0
x_odom = 0

x_p = 0
y_p = 0
z_o = 0
h_p = 0

def controlador_proporcional(q_ref, q, kp, sat, death_zone):
	err=q_ref-q
	tao=kp*err
	if abs(err)<=death_zone:
		tao=0
	if tao>sat:
		tao=sat
	if tao<-sat:
		tao=-sat
	return tao

#Callback de pose y orientacion simulador
def odom_callback(data):
    global x_odom, y_odom
    x_odom = data.pose.pose.position.x
    y_odom = data.pose.pose.position.y

def height_callback(data):
    global h_p
    h_p = data.altitude

def enviar_velocidad(vx,vy,vz,vaz):
	vel_msg = Twist()
	vel_msg.linear.x = float(vx)
	vel_msg.linear.y = float(vy)
	vel_msg.linear.z = float(vz)
	vel_msg.angular.z = float(vaz)
	vel_pub.publish(vel_msg)


def trayectoria():
    
    global vel_pub

    takeoff_pub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
    land_pub = rospy.Publisher("/bebop/land", Empty, queue_size=10)

    vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=10)
    pose_sub = rospy.Subscriber("/bebop/odom", Odometry , odom_callback)

    height_sub = rospy.Subscriber("/sonar_height", Range, height_callback)
    
    
    rospy.init_node('Trayectoria', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 1
    while not rospy.is_shutdown():
        print "y : "+ str(y_odom)
        if( y_odom < -4):
            land_pub.publish(Empty())
        else:
            enviar_velocidad(0.3,0.0,0.0,0.0)
        # opt = input("1 Despegue 3 Aterrizando")
        # print "x : "+ str(x)
        # #print "X : " + "{0:.2f}".format(x_p) + " Y : "+ "{0:.2f}".format(y_p) + " Oz : " + "{0:.2f}".format(z_o) + " Z : "  + str(h_p) 
        # if( opt == 1 ):
        #     print "Despegando \n"
        #     takeoff_pub.publish(Empty())
        #     # opt = 2
        # elif( opt == 2):
        #     print "Hover \n"
        #     enviar_velocidad(1.0,0.0,0.0,0.0)
        # elif ( opt == 3):
        #     print "Aterrizando \n"
        #     land_pub.publish(Empty())
        # elif ( opt == 4):
        #     print "Hover \n"
        #     enviar_velocidad(0.0,0.0,0.0,0.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        trayectoria()
    except rospy.ROSInterruptException:
        pass