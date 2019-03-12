#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Int32
#Librerias para pose ardrone en simulador
from tf2_msgs.msg import TFMessage
h_p = 0
x_p = 0
y_p = 0
z_o = 0
cnt = 0
mx = 0
my = 0

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
def pose_callback(data):
    global x_p, y_p, z_o
    x_p = data.transforms[0].transform.translation.x
    y_p = data.transforms[0].transform.translation.y
    z_o = data.transforms[0].transform.rotation.z

def cnt_callback(data):
    global cnt
    cnt = data.data

def my_callback(data):
    global my
    my = data.data

def mx_callback(data):
    global mx
    mx = data.data

def height_callback(data):
    global h_p
    h_p = data.range

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
    height_sub = rospy.Subscriber("/sonar_height", Range, height_callback)

    contornos_sub = rospy.Subscriber("/Contorno", Int32, cnt_callback)
    MX_sub = rospy.Subscriber("/MX", Int32, mx_callback)
    MY_sub = rospy.Subscriber("/MY", Int32, my_callback)
    
    
    rospy.init_node('Trayectoria', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    opt = 2
    i = 1
    step = 1
    while not rospy.is_shutdown():
        print "X : " + "{0:.2f}".format(x_p) + " Y : "+ "{0:.2f}".format(y_p) + " Oz : " + "{0:.2f}".format(z_o) + " Z : "  + str(h_p) + " " + str(cnt)
        if( opt == 1 ):
            print "Despegando \n"
            takeoff_pub.publish(Empty())
            # opt = 2
        elif( cnt == 0):
            takeoff_pub.publish(Empty())
            if(i == 1):
                if(y_p < 2):
                    vx = controlador_proporcional(0,x_p,1,0.3,0.05)
                    vz = controlador_proporcional(1.5, h_p, 1,0.3,0.05)
                    print "Parte 1"
                    enviar_velocidad(vx,0.5,vz,0.0)
                    i = 1
                else:
                    i = 2
            elif(i == 2):
                if(x_p < 4):
                    vy = controlador_proporcional(2,y_p,1,0.3,0.05)
                    vz = controlador_proporcional(1.5, h_p, 1,0.3,0.05)
                    enviar_velocidad(0.5,vy,vz,0.0)
                    print "Parte 2"
                    i = 2
                else:
                    i = 3
            elif(i == 3):
                if(y_p > -2):
                    vx = controlador_proporcional(4.0,x_p,1,0.3,0.05)
                    vz = controlador_proporcional(1.5, h_p, 1,0.3,0.05)
                    enviar_velocidad(vx,-0.5,vz,0.0)
                    print "Parte 3"
                    i = 2
                else:
                    i = 4 
            elif(i == 4):
                if(x_p > 0.20):
                    vy = controlador_proporcional(-2.0,y_p,1,0.3,0.05)
                    vz = controlador_proporcional(1.5, h_p, 1,0.3,0.05)
                    print "Parte 4"
                    enviar_velocidad(-0.5,vy,vz,0.0)
                    i = 4
                else:
                    print "Hover \n"
                    enviar_velocidad(0.0,0.0,0.0,0.0)
                    i = 1
            
        elif ( cnt == 1):
            if (step == 1):
                # vy = controlador_proporcional(319,mx,1,0.3,5)
                # vx = controlador_proporcional(240,my,1,0.3,5)
                vy = controlador_proporcional(317,mx,0.5,0.3,5)
                vx = controlador_proporcional(180,my,0.5,0.3,5)
                vz = controlador_proporcional(1, h_p, 1,0.3,0.05)
                enviar_velocidad(vx,vy,vz,0.0)
                print "Ajustando \n"
                print "MX : " + str(mx) +" MY : " + str(my)
                print "vx : " + "{0:.2f}".format(vx) + " vy : "+ "{0:.2f}".format(vy) + " vz : "  + "{0:.2f}".format(vz)
                ex = abs(317 - mx)
                ey = abs(180 - my)
                if((ex < 5) and (ey < 5)):
                    step = 2
                else:
                    step = 1
            elif( step == 2):
                print "Hover \n"
                enviar_velocidad(0.0,0.0,0.0,0.0)
                print "Aterrizando \n"
                land_pub.publish(Empty())

        elif ( opt == 1):
            print "Hover \n"
            enviar_velocidad(0.0,0.0,0.0,0.0)
        rate.sleep()

if __name__ == '__main__':
    try:
        trayectoria()
    except rospy.ROSInterruptException:
        pass