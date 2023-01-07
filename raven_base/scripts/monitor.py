
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
import numpy as np
# import tkinter as tk
# # from tkinter.ttk import *
# import matplotlib as plt
# # plt.matplotlib.use("TkAgg")
# from matplotlib.figure import Figure
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class MiscTable:
    def __init__(self, container_panel, container_row, container_col):
        self.x_pad = 6
        self.y_pad = 2
        self.panel = tk.Frame(container_panel, padx=0, pady=0, highlightbackground="black", highlightthickness=2)
        self.panel.grid(row=container_row, column=container_col, sticky="n")
        tk.Label(self.panel, text=f"clock", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12 bold", anchor="center").grid(row=0, column=0)
        tk.Label(self.panel, text=f"odom", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12 bold", anchor="center").grid(row=1, column=0)
        self.clock =tk.Label(self.panel, text="123456789.123", width=18, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12", anchor="e")
        self.clock.grid(row=0, column=1)
        self.odom =tk.Label(self.panel, text="-123.45, -1234.45", width=18, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12", anchor="e")
        self.odom.grid(row=1, column=1)

class ProximityTable:
    def set_value(obj, which, value):
        obj.values[which].config(text=str(value))

    def __init__(self, container_panel, container_row, container_col, name, rows):
        self.name = name
        self.rows = rows
        self.x_pad = 6
        self.y_pad = 2
        self.panel = tk.Frame(container_panel, padx=0, pady=0, highlightbackground="black", highlightthickness=2)
        self.panel.grid(row=container_row, column=container_col, sticky="n")
        self.counts = []
        self.rates = []
        self.values = []
        tk.Label(self.panel, text=f"{name}#", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12 bold", anchor="center").grid(row=0, column=0)
        tk.Label(self.panel, text="count", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12 bold", anchor="e").grid(row=0, column=1)
        tk.Label(self.panel, text="rate", width=6, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12 bold", anchor="e").grid(row=0, column=2)
        tk.Label(self.panel, text="range", width=8, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12 bold", anchor="e").grid(row=0, column=3)
        for i in range(rows):
            tk.Label(self.panel, text=f"{i}", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12 bold", anchor="center").grid(row=i+1, column=0)
            tmp =tk.Label(self.panel, text="1234", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12", anchor="e")
            self.counts.append(tmp)
            tmp.grid(row=i+1, column=1)
            tmp = tk.Label(self.panel, text="25.4", width=6, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12", anchor="e")
            self.rates.append(tmp)
            tmp.grid(row=i+1, column=2)
            tmp = tk.Label(self.panel, text="1.234", width=8, padx=self.x_pad, pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12", anchor="e")
            self.values.append(tmp)
            tmp.grid(row=i+1, column=3)

def create_circle(x, y, r, canvas): #center coordinates, radius
    x0 = x - r
    y0 = y - r
    x1 = x + r
    y1 = y + r
    return canvas.create_oval(x0, y0, x1, y1, fill="#004")

root = tk.Tk()
root.geometry("1024x1024")

btn = tk.Label(root, text='Raven data visualization', font='Helvetica 18 bold')
btn.grid(row=0, column=0, padx=0, pady=0)

lidar_panel = tk.Frame(root, highlightbackground="black", highlightthickness=1)
lidar_panel.grid(row=1, column=0)


# lidar_figure = plt.figure(dpi=100)
# lidar_plot = lidar_figure.add_subplot(1, 1, 1)
# lidar_plot.plot(0.5, 0.3, color="#C41E3A", marker="o", linestyle="")
# x = [ 0.1, 0.2, 0.3 ]
# y = [ -0.1, -0.2, -0.3 ]
# lidar_plot.plot(x, y, color="blue", marker=".", linestyle="")	# Plotting points
# canvas = FigureCanvasTkAgg(lidar_figure, master=lidar_panel)
# canvas.get_tk_widget().grid(row=1, column=0)
map_canvas = tk.Canvas(lidar_panel, width=1024, height=512)
map_canvas.grid(row=0, column=0)
create_circle((1024/2)-54, (512/2)-54, 2, map_canvas)
create_circle((1024/2)-54, (512/2)+54, 2, map_canvas)
# map_canvas.create_polygon(60,40, 30,50, 30,30, 60,40, fill="#ffa500")
map_canvas.create_arc((1024/2)-50, (512/2)-50, (1024/2)+50,(512/2)+50, start=173, extent=7, fill="#ffa500", style=tk.PIESLICE)

proximity_panel = tk.Frame(master=root)
proximity_panel.grid(row=2,column=0)
# toolbar = NavigationToolbar2Tk(canvas, toolbarFrame)
# btn2 = tk.Label(proximity_panel, text='col1').grid(row=0, column=0, padx=20, pady=10)

tofs = ProximityTable(proximity_panel, 0, 0, "tof", 8)
sonars = ProximityTable(proximity_panel, 0, 2, "sonar", 4)
sonars.set_value(1, 2.345)

misc = MiscTable(proximity_panel, 0, 3)


# Calling mainloop
root.mainloop()
