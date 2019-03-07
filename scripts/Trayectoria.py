#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#Librerias para pose ardrone en simulador
from tf2_msgs.msg import TFMessage

def rutina():
    i = 1
    if(i == 1):
        if(y_p < 2):
            print "1"
            enviar_velocidad(0.0,0.5,0.0,0.0)
            i = 1
        else:
            i = 2
    elif(i == 2):
        if(x_p < 2):
            print "2"
            enviar_velocidad(0.0,0.0,0.0,0.0)
            i = 2
        else:
            i = 3
    elif(i == 3):
        if(y_p > -2):
            print "3"
            enviar_velocidad(0.0,-0.5,0.0,0.0)
            i = 2
        else:
            print "4"
            enviar_velocidad(0.0,0.0,0.0,0.0)

#Callback de pose y orientacion simulador
def pose_callback(data):
    global x_p, y_p, z_o
    x_p = data.transforms[0].transform.translation.x
    y_p = data.transforms[0].transform.translation.y
    z_o = data.transforms[0].transform.rotation.z


def enviar_velocidad(vx,vy,vz,vaz):
	vel_msg = Twist()
	vel_msg.linear.x = float(vx)
	vel_msg.linear.y = float(vy)
	vel_msg.linear.z = float(vz)
	vel_msg.angular.x = float(0.0)
	vel_msg.angular.y = float(0.0)
	vel_msg.angular.z = float(vaz)
	vel_pub.publish(vel_msg)


def trayectoria():
    
    global vel_pub
    pub = rospy.Publisher('chatter', String, queue_size=10)

    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=10)

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    pose_sub = rospy.Subscriber("/tf", TFMessage , pose_callback)
 
    
    
    rospy.init_node('Trayectoria', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print "X : " + str(x_p) + " Y : "+ str(y_p) + " Oz : " + str(z_o)
        opt = input(" 1 Despegue 2 Aterrizar 3 Trayectoria 4 Hover : ")
        if( opt == 1 ):
            print "Despegando \n"
            takeoff_pub.publish(Empty())
        elif( opt == 2):
            print "Aterrizando \n"
            land_pub.publish(Empty())
        elif ( opt == 3):
            print "X : " + str(x_p) + " Y : "+ str(y_p) + " Oz : " + str(z_o)
            print "Trayectoria \n"
            rutina()
            #enviar_velocidad(1.0,0.0,0.0,0.0)
            # while (opt == 3):
            #     enviar_velocidad(1.0,0.0,0.0,0.0)
            #     opt = input(" 1 Despegue 2 Aterrizar 3 Trayectoria 4 Hover : ")
        elif ( opt == 4):
            print "Hover \n"
            enviar_velocidad(0.0,0.0,0.0,0.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        trayectoria()
    except rospy.ROSInterruptException:
        pass