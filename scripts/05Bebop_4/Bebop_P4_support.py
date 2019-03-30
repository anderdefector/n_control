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
pos_act=0
start=0
time_hover=5

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
    img = Control(0.15, 0.15, 0.15,frame)
    visualiza_img(img, w.img_rgb)
    visualiza_img(img, w.img_seg)

def Despegar():
    takeoff_pub.publish(Empty())

def Aterrizar():
    enviar_velocidad(0.0,0.0,0.0, 0.0)
    land_pub.publish(Empty())

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


def Control(kpx, kpy, kpz, imagen):
    global modo, t1, altitud, odom_var, odom_act, altura_act, pos_act, posiciones, start
    global control_x, control_y, control_z, control_yaw
    vX=0.0
    vY=0.0
    vZ=0.0
    saturar=0.1
    if modo==0:
        print 'inicia despegue'
        t1=time.time()
        Despegar()
        modo=1
        text='Despegando'
        cv2.putText(imagen, text, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), lineType=cv2.LINE_AA)
    if modo==1:
        if time.time()-t1>tiempo_espera_despegue: 
            modo=2
            start=1
        text='Despegando'
        cv2.putText(imagen, text, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), lineType=cv2.LINE_AA)            
    if modo==2:
        print 'ha despegado'
        print  '(' + str(posiciones[pos_act, 0])+ ' ' + str(posiciones[pos_act, 1]) + ' ' + str(posiciones[pos_act, 2]) +')'
        modo=3
    if modo==3:
        vX=control_x.control_PD(posiciones[pos_act, 0] , odom_var.pose.pose.position.x , kpx, saturar, 0.1)
        vY=control_y.control_PD(posiciones[pos_act, 1] , odom_var.pose.pose.position.y , kpy, saturar, 0.1)
        vZ=control_z.control_PD(posiciones[pos_act, 2] , altitud , 0.5, 0.5, 0.1)
        vR=control_yaw.control_PD(posiciones[pos_act, 3] , odom_var.pose.pose.orientation.z , kpz, saturar, 0.1)
        text='Punto ' + str(pos_act+1) #+ ': ('+ str(posiciones[pos_act, 0])+ ' ' + posiciones[pos_act, 1] + ' '+ posiciones[pos_act, 2]+')'
        cv2.putText(imagen, text, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), lineType=cv2.LINE_AA)            
        print '(' + str(odom_var.pose.pose.position.x)+ ' ' + str(odom_var.pose.pose.position.y) + ' ' + str(altitud) +')'
        print odom_var.pose.pose.position
        if (vX == 0.0 and vY == 0.0 and vZ == 0.0):
            modo=4
            t1=time.time()
            odom_act=odom_var
            alura_act=altitud
            print 'pasa a hover'
        enviar_velocidad(vX, vY, vZ,vR)

    if modo==4:
        vX=0.0#control_x.control_PD(odom_act.pose.pose.position.x , odom_var.pose.pose.position.x , kpx, saturar, 0.05)
        vY=0.0#control_y.control_PD(odom_act.pose.pose.position.y , odom_var.pose.pose.position.y , kpy, saturar, 0.05)
        vZ=0.0#control_z.control_PD(altura_act , altitud , 0.5, 0.2, 0.05)
        vR=0.0#control_yaw.control_PD(odom_act.pose.pose.orientation.z , odom_var.pose.pose.orientation.z , kpz, saturar, 0.05)
        text='Hover'
        cv2.putText(imagen, text, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), lineType=cv2.LINE_AA)            
        print '(' + str(odom_var.pose.pose.position.x)+ ' ' + str(odom_var.pose.pose.position.y) + ' ' + str(altitud) +')'
        if time.time()-t1>time_hover:
            modo=3
            pos_act=pos_act+1
            print 'pasa a siguieten punto', str(pos_act+1)
            if pos_act==len(posiciones):
                text='Aterrizando'
                cv2.putText(imagen, text, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), lineType=cv2.LINE_AA)                		    
                w.iniciar.invoke()
        enviar_velocidad(vX, vY, vZ,vR)

    if modo==-1:
        print 'Aterrizando'
        text='Aterrizando'
        cv2.putText(imagen, text, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), lineType=cv2.LINE_AA)                            
        Aterrizar()
        modo=-2
    if modo ==-2:
        text='Esperando'
        cv2.putText(imagen, text, (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 0, 255), lineType=cv2.LINE_AA)                            
    w.cModo_et.configure(text = str(modo))
    #w.cX_et.configure(text= str(x))
    #w.cY_et.configure(text= str(y))
    #w.cR_et.configure(text= str(r))
    w.cvX_et.configure(text= str(vX))
    w.cvY_et.configure(text= str(vY))
    w.cvZ_et.configure(text= str(vZ))
    return imagen


def visualiza_img(img, label_img):
    if len(img.shape)==3:
        img=cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)
        Recorte = cv2.resize(img,(320,240))
        a = Image2.fromarray(Recorte)
        b = ImageTk.PhotoImage(image=a)
        label_img.configure(image=b)
        label_img._image_cache = b


def enviar_velocidad(vx,vy,vz,vaz):
	vel_msg = Twist()
	vel_msg.linear.x = float(vx)
	vel_msg.linear.y = float(vy)
	vel_msg.linear.z = float(vz)
	vel_msg.angular.x = float(0.0)
	vel_msg.angular.y = float(0.0)
	vel_msg.angular.z = float(vaz)
	vel_pub.publish(vel_msg)


def inicio_fn():
    global modo, pos_act
    if inicio.get() == '0':
        w.iniciar.configure(text='Inicio')
        modo=-1
    else:
        w.iniciar.configure(text='Detener')
        openfile_positions()
        pos_act=0
        modo=0
        print 'iniiooooooo'


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

def openfile_positions():
    global posiciones
    filename = tkFileDialog.askopenfilename(parent=root)
    files = open(filename)
    msg = files.readlines()
    files.close()
    posiciones=np.zeros((len(msg),4))
    for x in range(0,len(msg)):
        p=msg[x].rsplit(' ')
        posiciones[x,0]=p[0]
        posiciones[x,1]=p[1]
        posiciones[x,2]=p[2]
        posiciones[x,3]=p[3]
    print posiciones

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
    import Bebop_P4
    Bebop_P4.vera_Bebop_P4_gui()
