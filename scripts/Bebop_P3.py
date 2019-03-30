#! /usr/bin/env python
#  -*- coding: utf-8 -*-


import sys
import Tkinter as tk
import Bebop_P3_support

import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged

def vera_Bebop_P3_gui():
    global val, w, root
    topico_imagen=str(sys.argv[1])
    root = tk.Tk()
    Bebop_P3_support.set_Tk_var()
    top = Toplevel1 (root)
    Bebop_P3_support.init(root, top)
    rospy.init_node("Interface", anonymous=False)
    rospy.Subscriber(topico_imagen, Image, Bebop_P3_support.receive)
    rospy.Subscriber("/bebop/odom", Odometry , Bebop_P3_support.receive_odom)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, Bebop_P3_support.receive_altitud)
    rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged, Bebop_P3_support.receive_beteria)
    root.mainloop()

w = None
def create_Toplevel1(root, *args, **kwargs):
    global w, w_win, rt
    rt = root
    w = tk.Toplevel (root)
    Bebop_P3_support.set_Tk_var()
    top = Toplevel1 (w)
    Bebop_P3_support.init(w, top, *args, **kwargs)
    return (w, top)

def destroy_Toplevel1():
    global w
    w.destroy()
    w = None

class Toplevel1:
    def __init__(self, top=None):

        top.geometry("650x560+399+94")
        top.title("Interface Circulos")

        self.img_rgb = tk.Label(top)
        self.img_rgb.place(x=300, y=10, height=240, width=320)

        self.img_seg = tk.Label(top)
        self.img_seg.place(x=300, y=260, height=240, width=320)

        self.min1_sc = tk.Scale(top, from_=0.0, to=255.0)
        self.min1_sc.place(x=30, y=10, width=200, height=21, bordermode='ignore')
        self.min1_sc.configure(length="200")
        self.min1_sc.configure(orient="horizontal")
        self.min1_sc.configure(showvalue="0")
        self.min1_sc.configure(variable=Bebop_P3_support.min1)

        self.min1_et = tk.Label(top)
        self.min1_et.place(x=5, y=10, height=18, width=20)
        self.min1_et.configure(text='r')

        self.min1_val_et = tk.Label(top)
        self.min1_val_et.place(x=235, y=10, height=18, width=40)
        self.min1_val_et.configure(textvariable=Bebop_P3_support.min1)

        self.min2_sc = tk.Scale(top, from_=0.0, to=255.0)
        self.min2_sc.place(x=30, y=40, width=200, height=21, bordermode='ignore')
        self.min2_sc.configure(length="200")
        self.min2_sc.configure(orient="horizontal")
        self.min2_sc.configure(showvalue="0")
        self.min2_sc.configure(variable=Bebop_P3_support.min2)

        self.min2_et = tk.Label(top)
        self.min2_et.place(x=5, y=40, height=18, width=20)
        self.min2_et.configure(text='g')

        self.min2_val_et = tk.Label(top)
        self.min2_val_et.place(x=235, y=40, height=18, width=40)
        self.min2_val_et.configure(textvariable=Bebop_P3_support.min2)

        self.min3_sc = tk.Scale(top, from_=0.0, to=255.0)
        self.min3_sc.place(x=30, y=70, width=200, height=21, bordermode='ignore')
        self.min3_sc.configure(length="200")
        self.min3_sc.configure(orient="horizontal")
        self.min3_sc.configure(showvalue="0")
        self.min3_sc.configure(variable=Bebop_P3_support.min3)

        self.min3_et = tk.Label(top)
        self.min3_et.place(x=5, y=70, height=18, width=20)
        self.min3_et.configure(text='b')

        self.min3_val_et = tk.Label(top)
        self.min3_val_et.place(x=235, y=70, height=18, width=40)
        self.min3_val_et.configure(textvariable=Bebop_P3_support.min3)

        self.max1_sc = tk.Scale(top, from_=0.0, to=255.0)
        self.max1_sc.place(x=30, y=100, width=200, height=21, bordermode='ignore')
        self.max1_sc.configure(length="200")
        self.max1_sc.configure(orient="horizontal")
        self.max1_sc.configure(showvalue="0")
        self.max1_sc.configure(variable=Bebop_P3_support.max1)

        self.max1_et = tk.Label(top)
        self.max1_et.place(x=5, y=100, height=18, width=20)
        self.max1_et.configure(text='R')

        self.max1_val_et = tk.Label(top)
        self.max1_val_et.place(x=235, y=100, height=18, width=40)
        self.max1_val_et.configure(textvariable=Bebop_P3_support.max1)

        self.max2_sc = tk.Scale(top, from_=0.0, to=255.0)
        self.max2_sc.place(x=30, y=130, width=200, height=21, bordermode='ignore')
        self.max2_sc.configure(length="200")
        self.max2_sc.configure(orient="horizontal")
        self.max2_sc.configure(showvalue="0")
        self.max2_sc.configure(variable=Bebop_P3_support.max2)

        self.max2_et = tk.Label(top)
        self.max2_et.place(x=5, y=130, height=18, width=20)
        self.max2_et.configure(text='G')

        self.max2_val_et = tk.Label(top)
        self.max2_val_et.place(x=235, y=130, height=18, width=40)
        self.max2_val_et.configure(textvariable=Bebop_P3_support.max2)

        self.max3_sc = tk.Scale(top, from_=0.0, to=255.0)
        self.max3_sc.place(x=30, y=160, width=200, height=21, bordermode='ignore')
        self.max3_sc.configure(length="200")
        self.max3_sc.configure(orient="horizontal")
        self.max3_sc.configure(showvalue="0")
        self.max3_sc.configure(variable=Bebop_P3_support.max3)

        self.max3_et = tk.Label(top)
        self.max3_et.place(x=5, y=160, height=18, width=20)
        self.max3_et.configure(text='B')

        self.max3_val_et = tk.Label(top)
        self.max3_val_et.place(x=235, y=160, height=18, width=40)
        self.max3_val_et.configure(textvariable=Bebop_P3_support.max3)

        self.radio_sc = tk.Scale(top, from_=1.0, to=400.0)
        self.radio_sc.place(x=30, y=190, width=200, height=21, bordermode='ignore')
        self.radio_sc.configure(length="200")
        self.radio_sc.configure(orient="horizontal")
        self.radio_sc.configure(showvalue="0")
        self.radio_sc.configure(variable=Bebop_P3_support.radio)
	self.radio_sc.set(80)

        self.radio_et = tk.Label(top)
        self.radio_et.place(x=5, y=190, height=18, width=20)
        self.radio_et.configure(text='Ra')

        self.radio_val_et = tk.Label(top)
        self.radio_val_et.place(x=235, y=190, height=18, width=40)
        self.radio_val_et.configure(textvariable=Bebop_P3_support.radio)

        self.cX_sc = tk.Scale(top, from_=1.0, to=640.0)
        self.cX_sc.place(x=30, y=220, width=200, height=21, bordermode='ignore')
        self.cX_sc.configure(length="200")
        self.cX_sc.configure(orient="horizontal")
        self.cX_sc.configure(showvalue="0")
        self.cX_sc.configure(variable=Bebop_P3_support.cenX)
        self.cX_sc.set(320)

        self.cX_et = tk.Label(top)
        self.cX_et.place(x=5, y=220, height=18, width=20)
        self.cX_et.configure(text='cX')

        self.cX_val_et = tk.Label(top)
        self.cX_val_et.place(x=235, y=220, height=18, width=40)
        self.cX_val_et.configure(textvariable=Bebop_P3_support.cenX)

        self.cY_sc = tk.Scale(top, from_=1.0, to=480.0)
        self.cY_sc.place(x=30, y=250, width=200, height=21, bordermode='ignore')
        self.cY_sc.configure(length="200")
        self.cY_sc.configure(orient="horizontal")
        self.cY_sc.configure(showvalue="0")
        self.cY_sc.configure(variable=Bebop_P3_support.cenY)
	self.cY_sc.set(240)

        self.cY_et = tk.Label(top)
        self.cY_et.place(x=5, y=250, height=18, width=20)
        self.cY_et.configure(text='cY')

        self.cY_val_et = tk.Label(top)
        self.cY_val_et.place(x=235, y=250, height=18, width=40)
        self.cY_val_et.configure(textvariable=Bebop_P3_support.cenY)

        self.modo_ch = tk.Checkbutton(top)
        self.modo_ch.place(x=30, y=370, height=28, width=100)
        self.modo_ch.configure(indicatoron="0")
        self.modo_ch.configure(text='RGB')
        self.modo_ch.configure(variable=Bebop_P3_support.modo_color)
        self.modo_ch.configure(command=Bebop_P3_support.change_mode)

        self.iniciar = tk.Checkbutton(top)
        self.iniciar.place(x=150, y=370, height=28, width=100)
        self.iniciar.configure(indicatoron="0")
        self.iniciar.configure(text='Inicio')
        self.iniciar.configure(variable=Bebop_P3_support.inicio)
        self.iniciar.configure(command=Bebop_P3_support.inicio_fn)


        self.Guardar = tk.Button(top)
        self.Guardar.place(x=30, y=408, height=28, width=100)
        self.Guardar.configure(command=Bebop_P3_support.Guardar)
        self.Guardar.configure(text='Guardar')

        self.cX_et = tk.Label(top)
        self.cX_et.place(x=30, y=436, height=28, width=60)
        self.cX_et.configure(foreground="#ff0000")

        self.cY_et = tk.Label(top)
        self.cY_et.place(x=100, y=436, height=28, width=60)
        self.cY_et.configure(foreground="#ff0000")

        self.cR_et = tk.Label(top)
        self.cR_et.place(x=170, y=436, height=28, width=60)
        self.cR_et.configure(foreground="#ff0000")

        self.cvX_et = tk.Label(top)
        self.cvX_et.place(x=30, y=464, height=28, width=60)
        self.cvX_et.configure(foreground="#0000ff")

        self.cvY_et = tk.Label(top)
        self.cvY_et.place(x=100, y=464, height=28, width=60)
        self.cvY_et.configure(foreground="#0000ff")

        self.cvZ_et = tk.Label(top)
        self.cvZ_et.place(x=170, y=464, height=28, width=60)
        self.cvZ_et.configure(foreground="#0000ff")

        self.cModo_et = tk.Label(top)
        self.cModo_et.place(x=50, y=492, height=28, width=160)
        self.cModo_et.configure(foreground="#ff00ff")

        self.menubar = tk.Menu(top)
        top.configure(menu = self.menubar)

        self.sub_menu = tk.Menu(top,tearoff=0)
        self.menubar.add_cascade(menu=self.sub_menu,label="Archivo")
        self.sub_menu.add_command(label="Guardar", command=Bebop_P3_support.Guardar)
        self.sub_menu.add_command(label="Abrir", command=Bebop_P3_support.Abrir)
        self.sub_menu1 = tk.Menu(top,tearoff=0)
        self.menubar.add_cascade(menu=self.sub_menu1, label="Ayuda")
        self.sub_menu1.add_command(label="Acerca de ...")


if __name__ == '__main__':
    vera_Bebop_P3_gui()




