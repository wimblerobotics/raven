from datetime import datetime
import tkinter as tk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg._range import Range
from std_msgs.msg import String
from threading import Thread, Lock
# import tkinter as tk
# # from tkinter.ttk import *
# import matplotlib as plt
# # plt.matplotlib.use("TkAgg")
# from matplotlib.figure import Figure
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

ros_mutex = Lock()


class TofSubscriberNode(Node):

    def __init__(self):
        super().__init__('tof_subscriber_node')
        self.number_sensors = 8  # There are 8 sensors.
        self.start_time = datetime.now()
        self.callback_count = np.zeros(self.number_sensors, dtype='int32')

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

    # Process a time-of-flight sensor message of type Range.

    def listener_callback(self, msg):
        global ros_mutex
        ros_mutex.acquire()
        # Get the sensor number.
        sensor_number = int(msg.header.frame_id[-1])
        range_value = msg.range
        self.callback_count[sensor_number] = self.callback_count[sensor_number] + 1
        global tofs
        tofs.set_value(sensor_number, "%1.3f" % range_value)
        tofs.set_count(sensor_number, "%d" % self.callback_count[sensor_number])

        if (self.callback_count[sensor_number] % 24) == 0:
            # Print out the frames per second of sensor data for all 8 sensors since the last plot update.
            # Divide by 8 if you want to know the frames per second per sensor.
            duration = datetime.now() - self.start_time
            fps = self.callback_count[sensor_number] / \
                (duration.seconds + (duration.microseconds / 1000000.0))
            tofs.set_rate(sensor_number, "%2.1f" % fps)

        ros_mutex.release()

class SonarSubscriberNode(Node):

    def __init__(self):
        super().__init__('sonar_subscriber_node')
        self.number_sensors = 4  # There are 4 sensors.
        self.start_time = datetime.now()
        self.callback_count = np.zeros(self.number_sensors, dtype='int32')

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

    # Process a time-of-flight sensor message of type Range.

    def listener_callback(self, msg):
        global ros_mutex
        ros_mutex.acquire()
        # Get the sensor number.
        sensor_number = int(msg.header.frame_id[-1])
        range_value = msg.range
        self.callback_count[sensor_number] = self.callback_count[sensor_number] + 1
        global sonars
        sonars.set_value(sensor_number, "%1.3f" % range_value)
        sonars.set_count(sensor_number, "%d" % self.callback_count[sensor_number])

        if (self.callback_count[sensor_number] % 24) == 0:
            # Print out the frames per second of sensor data for all 8 sensors since the last plot update.
            # Divide by 8 if you want to know the frames per second per sensor.
            duration = datetime.now() - self.start_time
            fps = self.callback_count[sensor_number] / \
                (duration.seconds + (duration.microseconds / 1000000.0))
            sonars.set_rate(sensor_number, "%2.1f" % fps)

        ros_mutex.release()

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


def create_circle(x, y, r, canvas):  # center coordinates, radius
    x0 = x - r
    y0 = y - r
    x1 = x + r
    y1 = y + r
    return canvas.create_oval(x0, y0, x1, y1, fill="#004")

# See: https://stackoverflow.com/questions/16745507/tkinter-how-to-use-threads-to-preventing-main-event-loop-from-freezing
class BackgroundTask():

    def __init__( self, taskFuncPointer ):
        self.__taskFuncPointer_ = taskFuncPointer
        self.__workerThread_ = None
        self.__isRunning_ = False

    def taskFuncPointer( self ) : return self.__taskFuncPointer_

    def isRunning( self ) : 
        return self.__isRunning_ and self.__workerThread_.isAlive()

    def start( self ): 
        if not self.__isRunning_ :
            self.__isRunning_ = True
            self.__workerThread_ = self.WorkerThread( self )
            self.__workerThread_.start()

    def stop( self ) : self.__isRunning_ = False

    class WorkerThread(Thread ):
        def __init__( self, bgTask ):      
            Thread.__init__( self )
            self.__bgTask_ = bgTask

        def run( self ):
            try :
                self.__bgTask_.taskFuncPointer()()
            except Exception as e:
                print(e)
            self.__bgTask_.stop()

def sonar_thread():
    sonar_subscriber = SonarSubscriberNode()
    rclpy.spin(sonar_subscriber)
    
def tof_thread():
    tof_subscriber = TofSubscriberNode()
    rclpy.spin(tof_subscriber)
    
def executor_thread():
    rclpy.spin()

def main(args=None):
    rclpy.init(args=args)
    global executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(SonarSubscriberNode())
    executor.add_node(TofSubscriberNode())
    e_thread = BackgroundTask(executor.spin)
    # tof_bg_thread = BackgroundTask(tof_thread)
    # sonar_bg_thread = BackgroundTask(sonar_thread)
    
    root = tk.Tk()
    root.geometry("1024x1024")

    btn = tk.Label(root, text='Raven data visualization',
                   font='Helvetica 18 bold')
    btn.grid(row=0, column=0, padx=0, pady=0)

    lidar_panel = tk.Frame(
        root, highlightbackground="black", highlightthickness=1)
    lidar_panel.grid(row=1, column=0)

    map_canvas = tk.Canvas(lidar_panel, width=1024, height=512)
    map_canvas.grid(row=0, column=0)
    create_circle((1024/2)-54, (512/2)-54, 2, map_canvas)
    create_circle((1024/2)-54, (512/2)+54, 2, map_canvas)
    # map_canvas.create_polygon(60,40, 30,50, 30,30, 60,40, fill="#ffa500")
    map_canvas.create_arc((1024/2)-50, (512/2)-50, (1024/2)+50, (512/2)+50,
                          start=173, extent=7, fill="#ffa500", style=tk.PIESLICE)

    proximity_panel = tk.Frame(master=root)
    proximity_panel.grid(row=2, column=0)

    global tofs, sonars
    tofs = ProximityTable(proximity_panel, 0, 0, "tof", 8)
    sonars = ProximityTable(proximity_panel, 0, 2, "sonar", 4)
    sonars.set_value(1, 2.345)

    misc = MiscTable(proximity_panel, 0, 3)

    # tof_bg_thread.start()
    # sonar_bg_thread.start()
    e_thread.start()
    #Calling mainloop
    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
