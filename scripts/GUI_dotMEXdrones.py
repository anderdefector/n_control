#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Tkinter as tk 
from Tkinter import *
Form = tk.Tk() 
Form.title('dotMEX Drones') 



btnTakeOff = tk.Button(Form, text='Take Off', width=25, command=Form.destroy) 
btnTakeOff.pack()
btnLand = tk.Button(Form, text='Land', width=25, command=Form.destroy) 
btnLand.pack()
btnHover = tk.Button(Form, text='Hover', width=25, command=Form.destroy) 
btnHover.pack()
w = Scale(Form, from_=10, to=-70) 
w.pack() 

Form.mainloop() 
