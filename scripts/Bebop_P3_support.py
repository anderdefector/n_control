#! /usr/bin/env python
#  -*- coding: utf-8 -*-
import math
import sys
import Tkinter as tk
import tkFileDialog
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged

from PIL import Image as Image2
from PIL import ImageTk
import time

from std_msgs.msg import Int16
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
br=CvBridge()

modo=-3   # modo en el control general
tiempo_espera_despegue=10 #segundos de espera despues de despegar
odom_var=Odometry()
odom_act=Odometry()
altura_act=0.0
altitud=0.0
t1=0

def receive_altitud(data):
    global altitud
    altitud = data.altitude
    print 'altura='+str(altitud)

def receive_beteria(data):
    global bateria
    bateria = data.percent
    print 'bat =' + str(bateria)

def receive_odom(data):
    global odom_var
    odom_var=data

def receive(data):
	global frame, cenX, cenY, radio
	frame = br.imgmsg_to_cv2(data, "bgr8")
	x, y, r =imagen_circulos(frame)
	Control(x, y, r, 0.001, 0.001, 0.001, 0.1, 0.05, 0.1, cenX.get(), cenY.get(), radio.get())

def Despegar():
    takeoff_pub.publish(Empty())

def Aterrizar():
    enviar_velocidad(0.0,0.0,0.0, 0.0)
    land_pub.publish(Empty())

def Control(x, y, r, kpx1, kpy1, kpz1,kpx, kpy, kpz, cox, coy, radius):
    global modo, t1, altitud, odom_var, odom_act, altura_act
    global control_x, control_y, control_z, control_yaw
    vX=0
    vY=0.0
    vZ=0.0
    if modo==0:
        print 'inicia despegue'
        t1=time.time()
        Despegar()
        modo=1
    if modo==1:
        if time.time()-t1>tiempo_espera_despegue: 
            modo=2
    if modo==2:
        print 'ha despegado'
        modo=3
    if modo==3:
        print 'Tracking cx='+str(x)+', cy='+str(y) + ', r='+str(r)
       	print cox
        if r!=-1:
            vX=control_x.control_P(radius,  r,kpx1, 0.1, 10)
            vZ=0.0#control_z.control_P(coy, y,kpz1, 0.1, 10)
            vY=control_y.control_P(cox, x,kpy1, 0.1, 10)
            if (vX == 0.0 and vY == 0.0 and vZ == 0.0):
                modo=4
                odom_act=odom_var
                altura_act=altitud
        enviar_velocidad(vX, vY, 0.0,0.0)
    if modo==4:
        vX=control_x.control_PD(odom_act.pose.pose.position.x , odom_var.pose.pose.position.x , kpx, 0.1, 0.1)
        vY=control_y.control_PD(odom_act.pose.pose.position.y - 1.0, odom_var.pose.pose.position.y , kpy, 0.1, 0.1)
        vZ=0.0#control_z.control_PD(altura_act , altitud , kpz, 0.1, 0.05)
        vR=control_yaw.control_PD(odom_act.pose.pose.orientation.z , odom_var.pose.pose.orientation.z , kpz, 0.1, 0.1)
        if (vX == 0.0 and vY == 0.0 and vZ == 0.0):
            modo=5
            odom_act=odom_var
            altura_act=altitud
        enviar_velocidad(vX, vY, 0.0,vR)
        print 'xa=' + str(odom_act.pose.pose.position.x) + 'ya=' + str(odom_act.pose.pose.position.y) + 'za=' + str(odom_act.pose.pose.position.z)
        print 'x=' + str(odom_var.pose.pose.position.x) + 'y=' + str(odom_var.pose.pose.position.y) + 'z=' + str(odom_var.pose.pose.position.z)
        

    if modo==5:
        vX=control_x.control_PD(odom_act.pose.pose.position.x + 1.0 , odom_var.pose.pose.position.x , kpx, 0.1, 0.1)
        vY=control_y.control_PD(odom_act.pose.pose.position.y , odom_var.pose.pose.position.y , kpy, 0.1, 0.1)
        vZ=control_z.control_PD(altura_act , altitud , kpz, 0.1, 0.1)
        vR=control_yaw.control_PD(odom_act.pose.pose.orientation.z , odom_var.pose.pose.orientation.z , kpz, 0.1, 0.1)
        print 'xa=' + str(odom_act.pose.pose.position.x) + 'ya=' + str(odom_act.pose.pose.position.y) + 'za=' + str(odom_act.pose.pose.position.z)
        print 'x=' + str(odom_var.pose.pose.position.x) + 'y=' + str(odom_var.pose.pose.position.y) + 'z=' + str(odom_var.pose.pose.position.z)
        if (vX == 0.0 and vY == 0.0 and vZ == 0.0):
            modo==-1
            odom_act=odom_var
        enviar_velocidad(vX, vY, 0.0,vR)

    if modo==-1:
        print 'Aterrizando'
        Aterrizar()
        modo=-2
    w.cModo_et.configure(text = str(modo))
    w.cX_et.configure(text= str(x))
    w.cY_et.configure(text= str(y))
    w.cR_et.configure(text= str(r))
    w.cvX_et.configure(text= str(vX))
    w.cvY_et.configure(text= str(vY))
    w.cvZ_et.configure(text= str(vZ))


def imagen_circulos(frame):
    x=-1
    y=-1
    r=-1
    kernel = np.ones((5,5),np.uint8)
    segmentada = segmentar(frame)
    segmentada = cv2.morphologyEx(segmentada, cv2.MORPH_OPEN, kernel)
    segmentada = cv2.erode(segmentada,kernel,iterations = 3)
    segmentada = cv2.dilate(segmentada,kernel,iterations = 3)
    segmentada = cv2.GaussianBlur(segmentada,(9,9),0)
    img_cir, x, y, r = contornos(segmentada, frame)
    #img_cir, x, y, r =detecta_circulos(segmentada, frame)
    #visualiza_img(frame, w.img_rgb)
    cv2.circle(img_cir, (cenX.get(), cenY.get()), 5, (255,255,0), -1)
    visualiza_img(img_cir, w.img_rgb)
    visualiza_img(segmentada, w.img_seg)
    return x, y, r

def contornos(seg, img):
	_, contours, hierachy= cv2.findContours(seg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	#cv2.drawContours(img, contours, -1, (0, 0, 255), 2, cv2.LINE_AA)
	area = 0
	radio=-1.0
	cXM=-1.0
	cYM=-1.0
	if len(contours) > 0:
		for c in contours:
			M = cv2.moments(c)
			if M["m00"] > area:
				area=M["m00"]
				cXM = int(M["m10"] / M["m00"])
				cYM = int(M["m01"] / M["m00"])
				contorno_importante=c
				radio=int(math.sqrt(area/3.1416))
		cv2.circle(img, (cXM, cYM), 7, (0,0,255), -1)
		cv2.circle(img, (cXM, cYM), radio, (0,255,0), 2)
	return img, cXM, cYM, radio


def visualiza_img(img, label_img):
	if len(img.shape)==3:
		img=cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)
	Recorte = cv2.resize(img,(320,240))
        a = Image2.fromarray(Recorte)
        b = ImageTk.PhotoImage(image=a)
        label_img.configure(image=b)
        label_img._image_cache = b

def segmentar(frame):
        lower = np.array([min1.get(),min2.get(),min3.get()])
        upper = np.array([max1.get(),max2.get(),max3.get()])
        img=frame.copy()
        if modo_color.get()=='1':
            img=cv2.cvtColor(frame.copy(), cv2.COLOR_RGB2HSV)
        segmentada = cv2.inRange(img, lower, upper)
	return segmentada

def enviar_velocidad(vx,vy,vz,vaz):
	vel_msg = Twist()
	vel_msg.linear.x = float(vx)
	vel_msg.linear.y = float(vy)
	vel_msg.linear.z = float(vz)
	vel_msg.angular.x = float(0.0)
	vel_msg.angular.y = float(0.0)
	vel_msg.angular.z = float(vaz)
	vel_pub.publish(vel_msg)


class controlador:
    def __init__(self):
        self.er_ant=0.0
    def control_P(self,ref,q,kp,sat,dz):
 	er=ref-q
	tao=kp*er
	if abs(er)<=dz:
		tao=0.0
	if tao>sat:
		tao=sat
	if tao<-sat:
		tao=-sat
        return tao

    def control_PD(self,ref,q,kp,sat,dz):
        kd=kp/10
 	er=ref-q
        der=er-self.er_ant
        self.er_ant=er
	tao=kp*er+kd*der
	if abs(er)<=dz:
		tao=0.0
	if tao>sat:
		tao=sat
	if tao<-sat:
		tao=-sat
        return tao

def inicio_fn():
    global modo
    if inicio.get() == '0':
        w.iniciar.configure(text='Inicio')
        modo=-1
    else:
        w.iniciar.configure(text='Detener')
        modo=0

def habilitar_scrolls(estado):
    w.min1_sc.configure(state=estado)
    w.min2_sc.configure(state=estado)
    w.min3_sc.configure(state=estado)
    w.max1_sc.configure(state=estado)
    w.max2_sc.configure(state=estado)
    w.max3_sc.configure(state=estado)
    w.radio_sc.configure(state=estado)
    w.cX_sc.configure(state=estado)
    w.cY_sc.configure(state=estado)


def set_Tk_var():
    global min1, min2, min3, max1, max2, max3, radio, cenX, cenY, p2, inicio, modo_color
    global takeoff_pub, land_pub, vel_pub
    global control_x, control_y, control_z, control_yaw
    min1 = tk.DoubleVar()
    min2 = tk.DoubleVar()
    min3 = tk.DoubleVar()
    max1 = tk.DoubleVar()
    max2 = tk.DoubleVar()
    max3 = tk.DoubleVar()
    radio   = tk.IntVar()
    cenX = tk.IntVar()
    cenY   = tk.IntVar()
    inicio = tk.StringVar()
    modo_color = tk.StringVar()
    inicio.set('0')
    modo_color.set('0')
    takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
    land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)        
    control_x=controlador()
    control_y=controlador()
    control_z=controlador()
    control_yaw=controlador()

def Guardar():
    if modo_color.get()=='':
        x='0'
    else:
        x=modo_color.get()
    files=tkFileDialog.asksaveasfile(mode='w',defaultextension=".txt")
    files.write(str(min1.get())+' '+str(min2.get())+' '+str(min3.get())+' '+str(max1.get())+' '+str(max2.get())+' '+str(max3.get())+' '+x +' '+str(radio.get())+' '+str(cenX.get())+' '+str(cenY.get()))
    files.close()
    print 'Archivo Guardado'

def Abrir():
    filename = tkFileDialog.askopenfilename(parent=root)
    files = open(filename)
    msg = files.readlines()
    Cal = msg[0].rsplit(' ')
    w.min1_sc.set(Cal[0])
    w.min2_sc.set(Cal[1])
    w.min3_sc.set(Cal[2])
    w.max1_sc.set(Cal[3])
    w.max2_sc.set(Cal[4])
    w.max3_sc.set(Cal[5])
    modo_color.set(Cal[6])
    w.radio_sc.set(Cal[7])
    w.cX_sc.set(Cal[8])
    w.cY_sc.set(Cal[9])
    print 'Calibracion abierta : '+ str(filename)

def change_mode():
    if modo_color.get() == '0':
        w.min1_et.configure(text='r')
        w.min2_et.configure(text='g')
        w.min3_et.configure(text='b')
        w.max1_et.configure(text='R')
        w.max2_et.configure(text='G')
        w.max3_et.configure(text='B')
        w.modo_ch.configure(text='RGB')
    else:
        w.min1_et.configure(text='h')
        w.min2_et.configure(text='s')
        w.min3_et.configure(text='v')
        w.max1_et.configure(text='H')
        w.max2_et.configure(text='S')
        w.max3_et.configure(text='V')
        w.modo_ch.configure(text='HSV')        

def init(top, gui, *args, **kwargs):
    global w, top_level, root
    w = gui
    top_level = top
    root = top

def destroy_window():
    global top_level
    top_level.destroy()
    top_level = None

if __name__ == '__main__':
    import Bebop_P3
    Bebop_P3.vera_Bebop_P3_gui()
