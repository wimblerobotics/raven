from datetime import datetime
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
import math
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg._laser_scan import LaserScan
from sensor_msgs.msg._range import Range
from std_msgs.msg import String
from threading import Thread, Lock

ros_mutex = Lock()

class TofSubscriberNode(Node):

    def __init__(self):
        super().__init__('tof_subscriber_node')
        self.number_sensors = 8  # There are 8 sensors.
        self.start_time = datetime.now()
        self.callback_counts = np.zeros(self.number_sensors, dtype='int32')
        self.rates = np.zeros(self.number_sensors, dtype='float32')
        self.values = np.zeros(self.number_sensors, dtype='float32')

        # Set up the ROS 2 quality of service in order to read the sensor data.
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to the sensor topics.
        for sensor_number in range(self.number_sensors):
            self.subscription = self.create_subscription(
                Range,
                '/tof{s}Sensor'.format(s=sensor_number),
                self.listener_callback,
                qos_profile,
            )

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global ros_mutex
        with ros_mutex:
            # Get the sensor number.
            sensor_number = int(msg.header.frame_id[-1])
            range_value = msg.range
            self.callback_counts[sensor_number] = self.callback_counts[sensor_number] + 1
            global tofs
            self.values[sensor_number] = range_value

            if (self.callback_counts[sensor_number] % 24) == 0:
                duration = datetime.now() - self.start_time
                fps = self.callback_counts[sensor_number] / \
                    (duration.seconds + (duration.microseconds / 1000000.0))
                self.rates[sensor_number] = fps

class SonarSubscriberNode(Node):

    def __init__(self):
        super().__init__('sonar_subscriber_node')
        self.number_sensors = 4  # There are 4 sensors.
        self.start_time = datetime.now()
        self.callback_counts = np.zeros(self.number_sensors, dtype='int32')
        self.rates = np.zeros(self.number_sensors, dtype='float32')
        self.values = np.zeros(self.number_sensors, dtype='float32')

        # Set up the ROS 2 quality of service in order to read the sensor data.
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to the sensor topics.
        for sensor_number in range(self.number_sensors):
            self.subscription = self.create_subscription(
                Range,
                '/sonar{s}Sensor'.format(s=sensor_number),
                self.listener_callback,
                qos_profile,
            )

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global ros_mutex
        with ros_mutex:
            # Get the sensor number.
            sensor_number = int(msg.header.frame_id[-1])
            range_value = msg.range
            self.callback_counts[sensor_number] = self.callback_counts[sensor_number] + 1
            global sonars
            self.values[sensor_number] = range_value

            global map_canvas
            scale = 512.0 / 12.0
            x = math.cos(math.pi * (sensor_number / 2)) * range_value * scale
            y = math.sin(math.pi * (sensor_number / 2)) * range_value * scale

            if (self.callback_counts[sensor_number] % 24) == 0:
                duration = datetime.now() - self.start_time
                fps = self.callback_counts[sensor_number] / \
                    (duration.seconds + (duration.microseconds / 1000000.0))
                self.rates[sensor_number] = fps

class LidarSubscriberNode(Node):

    def update_annot(self, ind):
        global scatter_plot
        pos = scatter_plot.get_offsets()[ind["ind"][0]]
        self.annot.xy = pos
        text = "{:1.3f}, {:1.3f}".format(pos[0], pos[1])
        self.annot.set_text(text)
        # self.annot.get_bbox_patch().set_facecolor('#eafff5')
        # self.annot.get_bbox_patch().set_alpha(0.4)

    def hover(self, event):
        global ax, fig, scatter_plot
        vis = self.annot.get_visible()
        if event.inaxes == ax:
            cont, ind = scatter_plot.contains(event)
            if cont:
                self.update_annot(ind)
                self.annot.set_visible(True)
                fig.canvas.draw_idle()
            else:
                if vis:
                    self.annot.set_visible(False)
                    fig.canvas.draw_idle()
                    
    def __init__(self):
        super().__init__('lidar_subscriber_node')

        global ax, fig
        self.xs = []
        self.ys = []
        self.ss = []
        self.setup_complete = False
        self.scatter_created = False
        
        # Set up the ROS 2 quality of service in order to read the sensor data.
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to the LIDAR topic.
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile,
        )

        # self.subscription  # prevent unused variable warning

        ax.grid(True)
        self.annot = ax.annotate("here is some annotation", xy=(10,10), xytext=(20,20),textcoords="offset points",
                            bbox=dict(boxstyle="round", fc="w"),
                            arrowprops=dict(arrowstyle="->"))
        # self.annot = ax.annotate("here is some annotation", xy=(10,10), xytext=(20,20),textcoords="offset points")
        self.annot.set_visible(True) ###

        fig.canvas.mpl_connect("motion_notify_event", self.hover)
                                
    def listener_callback(self, msg):
        # global map_canvas
        global ros_mutex
        with ros_mutex:
            angle = msg.angle_min
            scale = 1.0
            if not self.setup_complete:
                self.number_points = len(msg.ranges)
                self.xs = np.zeros(len(msg.ranges), dtype='float32')
                self.ys = np.zeros(len(msg.ranges), dtype='float32')
                self.ss = np.zeros(len(msg.ranges), dtype='float32')
                self.setup_complete = True

            angle = 0.0
            index = 0
            for range in msg.ranges:
                if (range > 50.0) or (range < -50.0):
                    range = 0.0
                x = math.cos(angle) * range * scale
                y = math.sin(angle) * range * scale
                self.xs[index] = x
                self.ys[index] = y
                self.ss[index] = 1.0
                angle = angle + msg.angle_increment
                index = index + 1

class MiscTable:
    def __init__(self, container_panel, container_row, container_col):
        self.x_pad = 6
        self.y_pad = 2
        self.panel = tk.Frame(container_panel, padx=0, pady=0,
                              highlightbackground="black", highlightthickness=2)
        self.panel.grid(row=container_row, column=container_col, sticky="n")
        tk.Label(self.panel, text=f"clock", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1,
                 relief="solid", font="Courier 12 bold", anchor="center").grid(row=0, column=0)
        tk.Label(self.panel, text=f"odom", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1,
                 relief="solid", font="Courier 12 bold", anchor="center").grid(row=1, column=0)
        self.clock = tk.Label(self.panel, text="123456789.123", width=18, padx=self.x_pad,
                              pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12", anchor="e")
        self.clock.grid(row=0, column=1)
        self.odom = tk.Label(self.panel, text="-123.45, -1234.45", width=18, padx=self.x_pad,
                             pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12", anchor="e")
        self.odom.grid(row=1, column=1)

class ProximityTable:
    def set_count(obj, which, value):
        obj.counts[which].config(text=str(value))

    def set_rate(obj, which, value):
        obj.rates[which].config(text=str(value))

    def set_value(obj, which, value):
        obj.values[which].config(text=str(value))
    
    def __init__(self, container_panel, container_row, container_col, name, rows):
        self.name = name
        self.rows = rows
        self.x_pad = 6
        self.y_pad = 2
        self.panel = tk.Frame(container_panel, padx=0, pady=0,
                              highlightbackground="black", highlightthickness=2)
        self.panel.grid(row=container_row, column=container_col, sticky="n")
        self.counts = []
        self.rates = []
        self.values = []
        tk.Label(self.panel, text=f"{name}#", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1,
                 relief="solid", font="Courier 12 bold", anchor="center").grid(row=0, column=0)
        tk.Label(self.panel, text="count", width=7, padx=self.x_pad, pady=self.y_pad,
                 borderwidth=1, relief="solid", font="Courier 12 bold", anchor="e").grid(row=0, column=1)
        tk.Label(self.panel, text="rate", width=6, padx=self.x_pad, pady=self.y_pad,
                 borderwidth=1, relief="solid", font="Courier 12 bold", anchor="e").grid(row=0, column=2)
        tk.Label(self.panel, text="range", width=8, padx=self.x_pad, pady=self.y_pad,
                 borderwidth=1, relief="solid", font="Courier 12 bold", anchor="e").grid(row=0, column=3)
        for i in range(rows):
            tk.Label(self.panel, text=f"{i}", width=7, padx=self.x_pad, pady=self.y_pad, borderwidth=1,
                     relief="solid", font="Courier 12 bold", anchor="center").grid(row=i+1, column=0)
            tmp = tk.Label(self.panel, text="1234", width=7, padx=self.x_pad, pady=self.y_pad,
                           borderwidth=1, relief="solid", font="Courier 12", anchor="e")
            self.counts.append(tmp)
            tmp.grid(row=i+1, column=1)
            tmp = tk.Label(self.panel, text="25.4", width=6, padx=self.x_pad, pady=self.y_pad,
                           borderwidth=1, relief="solid", font="Courier 12", anchor="e")
            self.rates.append(tmp)
            tmp.grid(row=i+1, column=2)
            tmp = tk.Label(self.panel, text="1.234", width=8, padx=self.x_pad,
                           pady=self.y_pad, borderwidth=1, relief="solid", font="Courier 12", anchor="e")
            self.values.append(tmp)
            tmp.grid(row=i+1, column=3)

def update_lidar():
    global lidar_subscriber, lidar_subscriber_exists, ax, fig, scatter_plot
    if lidar_subscriber_exists:
        xs = lidar_subscriber.xs
        ys = lidar_subscriber.ys
        ss = lidar_subscriber.ss
        # ax.cla()
        
        if lidar_subscriber.scatter_created:
            scatter_plot.remove()
        scatter_plot = ax.scatter(xs, ys, ss, color='r')
        lidar_subscriber.scatter_created = True
        ax.set_xlim([-6, 6])
        ax.set_ylim([-6, 6])
        # ax.draw()
        # ax.pause(0.01)
        fig.canvas.draw()
        fig.canvas.flush_events()
    root.after(500, update_lidar)

def update_sonar():
    global sonar_subscriber, sonar_subscriber_exists, sonars
    if sonar_subscriber_exists:
        for which in range(sonar_subscriber.number_sensors):
            sonars.counts[which].config(text="%d" % sonar_subscriber.callback_counts[which])
            sonars.rates[which].config(text="%1.3f" % sonar_subscriber.rates[which])
            sonars.values[which].config(text="%1.3f" % sonar_subscriber.values[which])
    root.after(30, update_sonar)


def update_tof():
    global tof_subscriber, tof_subscriber_exists, tofs
    if tof_subscriber_exists:
        for which in range(tof_subscriber.number_sensors):
            tofs.counts[which].config(text="%d" % tof_subscriber.callback_counts[which])
            tofs.rates[which].config(text="%1.3f" % tof_subscriber.rates[which])
            tofs.values[which].config(text="%1.3f" % tof_subscriber.values[which])
    root.after(30, update_tof)


def main(args=None):
    global lidar_subscriber_exists, sonar_subscriber_exists, tof_subscriber_exists
    lidar_subscriber_exists  = False
    sonar_subscriber_exists = False
    tof_subscriber_exists = False
    
    rclpy.init(args=args)
    
    global root
    root = tk.Tk()
    root.geometry("1024x1024")

    btn = tk.Label(root, text='Raven data visualization',
                   font='Helvetica 18 bold')
    btn.grid(row=0, column=0, padx=0, pady=0)

    lidar_panel = tk.Frame(
        root, highlightbackground="black", highlightthickness=1)
    lidar_panel.grid(row=1, column=0)

    global fig, ax, lidar_canvas
    fig = Figure(figsize=(8, 8), dpi=100)
    x_axis_half_range = 1024.0 / 20.0
    y_axis_half_range = 512.0 / 20.0
    plt.ion()
    plt.show()
    fig.add_axes([-x_axis_half_range, -y_axis_half_range, x_axis_half_range, y_axis_half_range])
    ax = fig.add_subplot(111)
    
    lidar_canvas = FigureCanvasTkAgg(fig, master=lidar_panel)
    lidar_canvas.get_tk_widget().grid(row=0, column=0)
    root.after(500, update_lidar)
    root.after(30, update_sonar)
    root.after(30, update_tof)

    proximity_panel = tk.Frame(master=root)
    proximity_panel.grid(row=2, column=0)

    global tofs, sonars
    tofs = ProximityTable(proximity_panel, 0, 0, "tof", 8)
    sonars = ProximityTable(proximity_panel, 0, 2, "sonar", 4)

    misc = MiscTable(proximity_panel, 0, 3)

    # global executor
    # See: https://answers.ros.org/question/377848/spinning-multiple-nodes-across-multiple-threads/
    global lidar_subscriber
    lidar_subscriber = LidarSubscriberNode()
    lidar_subscriber_exists = True
    global sonar_subscriber
    sonar_subscriber = SonarSubscriberNode()
    sonar_subscriber_exists = True
    global tof_subscriber
    tof_subscriber = TofSubscriberNode()
    tof_subscriber_exists = True
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(lidar_subscriber)
    executor.add_node(sonar_subscriber)
    executor.add_node(tof_subscriber)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    #Calling mainloop
    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
