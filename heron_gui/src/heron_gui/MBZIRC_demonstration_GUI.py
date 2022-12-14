#!/usr/bin/python3

import getopt, sys
import csv

import rospy
import numpy as np
import tkinter as tk
import message_filters
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
import heron_gui.config as CFG
from tf.transformations import quaternion_from_euler

class Heron_GUI(tk.Tk):
    def __init__(self, track_time):
        super().__init__()

        self.lla_data = [0.0, 0.0, 0.0]
        self.utm_data = [0.0, 0.0, 0.0]

        self._scat = None
        self.pos_data_prev = [0.0, 0.0, 0.0]
        self.pos_data = [0.0, 0.0, 0.0]

        self.local_x_arr = np.array([])
        self.local_y_arr = np.array([])
        self.waypoints = []
        self.track_time = int(track_time)
        self.track_plot_prev = None

        # Subscriber for GPS(LLA)
        rospy.Subscriber(CFG.GPS_SUBSCRIBER, NavSatFix, self.callback)
        self.pub = rospy.Publisher(CFG.WP_PUBLISHER, Path, queue_size=1)

        self.wp_points = []
        self.wp_annots = []
        self.create_gui(track_time)

    # CREATE GUI (change 'config.py' for customization)
    def create_gui(self, track_time):
        topFrame = tk.Frame(self, width=CFG.FRAME_SIZE, height=CFG.FRAME_SIZE)
        topFrame.pack_propagate(0)
        topFrame.pack(side=tk.TOP)
        
        # HEADER & POSITION VALUES to print (LLA & UTM)
        self.title = "Heron GPS Tracking & Waypoint Viewer GUI"
        self.label = ttk.Label(topFrame,
                            text="Latitude: {:.2f}\nLongitude: {:.2f}\nAltitude: {:.2f}\n\nUTM X: {:.2f}\nUTM Y: {:.2f}\nUTM Z: {:.2f}\n\nLocal X: {:.2f}\nLocal Y: {:.2f}\nLocal Z: {:.2f}" \
                                  .format(self.lla_data[0], self.lla_data[1], self.lla_data[2], \
                                          self.utm_data[0], self.utm_data[1], self.utm_data[2], \
                                          self.pos_data[0], self.pos_data[1], self.pos_data[2]),
                            font=('Digital-7', 11),
                            anchor=tk.W,
                            background="#FFE08C",
                            width=20)
        self.label.pack(side=tk.RIGHT, fill=tk.Y)
        self.label.after(1, self.update)

        # MAP PLOT IMAGE VIEW
        height = 10
        self.fig = Figure(figsize=(height,height*CFG.MAP_WIDTH/CFG.MAP_HEIGHT))
        self.a = self.fig.add_subplot(111)
        im = plt.imread(CFG.MAP_PATH)

        self.a.imshow(im, extent=[0,CFG.MAP_WIDTH,0,CFG.MAP_HEIGHT], alpha=0.7)
        self.a.set_xlim(0, CFG.MAP_WIDTH)
        self.a.set_ylim(0, CFG.MAP_HEIGHT)
        self.a.set_title("GPS 2D Plot", fontsize=16)
        self.a.set_xlabel("x", fontsize=10)
        self.a.set_ylabel("y", fontsize=10)

        self.canvas = FigureCanvasTkAgg(self.fig, master=topFrame)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.Y)
        self.cid = self.canvas.mpl_connect('button_press_event', self.onclick)

        # Add Buttons
        self.btn_export = tk.Button(master=self, text="Publish WP List", command=lambda:self.wp_publish())
        self.btn_export.pack(side=tk.BOTTOM, fill=tk.X)

        self.btn_reset = tk.Button(master=self, text="RESET waypoints", command=lambda:self.wp_reset())
        self.btn_reset.pack(side=tk.BOTTOM, fill=tk.X)

    def onclick(self, event):
        global ix, iy
        ix, iy = event.xdata, event.ydata

        pos = [ix, iy, 0]
        utm = [sum(x) for x in zip(CFG.MAP_ORIGIN_UTM, pos)]

        # print('x = {}, y = {}'.format(utm[0], utm[1]))

        if len(self.waypoints) == CFG.MAX_WP2PLOT:
            # POP and PUSH
            self.waypoints.pop(0)
            # self.canvas.mpl_disconnect(self.cid)
        
        self.waypoints.append([utm[0], utm[1], utm[2]])
        self.draw_waypoints()

    def draw_waypoints(self):
        # Delete Waypoints drawn in previous frame
        for _ in self.wp_points:
            _.remove()
        for _ in self.wp_annots:
            _.remove()
        self.wp_points = []
        self.wp_annots = []

        # Draw new Waypoins 
        for idx, wp in enumerate(self.waypoints):
            if np.sum(wp) != 0:
                local_x = wp[0] - CFG.MAP_ORIGIN_UTM[0]
                local_y = wp[1] - CFG.MAP_ORIGIN_UTM[1]

                wp_point = self.a.scatter(local_x, local_y, color=CFG.WAYPOINT_COLOR, marker='.', edgecolors='none')
                wp_annot = self.a.annotate("{}".format(idx+1), (local_x, local_y), color="white")

                self.wp_points.append(wp_point)
                self.wp_annots.append(wp_annot)

    def wp_reset(self):
        self.waypoints = []
        self.draw_waypoints()

    # EXPORT CSV FILE of set waypoints from GUI
    def wp_publish(self):
        path = Path()

        # print(self.waypoints)
        for wp in self.waypoints:
            stamp = PoseStamped()
            # print(wp)
            stamp.pose.position.x = wp[0]
            stamp.pose.position.y = wp[1]
            stamp.pose.position.z = wp[2]

            q = quaternion_from_euler(0.0, 0.0, 0.0)
            stamp.pose.orientation = Quaternion(*q)

            path.poses.append(stamp)

            print(path)

        self.pub.publish(path)

    def callback(self, data_lla):
        # Set Current Position as LLA & UTM & Local_XYZ(the point position on map image (m))
        self.lla_data = [data_lla.latitude, data_lla.longitude, data_lla.altitude]
        self.lla_data = np.around(self.lla_data, decimals=CFG._PRECISION)


        self.utm_data = self.lla2utm(self.lla_data)
        self.utm_data = np.around(self.utm_data, decimals=CFG._PRECISION)

        self.pos_data_prev = self.pos_data
        self.pos_data = np.around(np.subtract(self.utm_data, CFG.MAP_ORIGIN_UTM), decimals=CFG._PRECISION)

        # Accumulate pose to show the Path on the map
        if len(self.local_x_arr) >= self.track_time:
            self.local_x_arr = np.delete(self.local_x_arr, 0)
            self.local_y_arr = np.delete(self.local_y_arr, 0)

        self.local_x_arr = np.append(self.local_x_arr, self.pos_data[0])
        self.local_y_arr = np.append(self.local_y_arr, self.pos_data[1])

    def lla2utm(self, lla_data):
        dLatitude = lla_data[0]
        dLongitude = lla_data[1]

        dLat = dLatitude * np.pi/180
        dLon = dLongitude * np.pi/180

        lon0_f = np.floor(dLongitude/6)*6+3
        lon0 = lon0_f*np.pi/180
        k0 = 0.9996
        FE = 500000
        FN = (dLatitude < 0)*10000000

        Wa = 6378137
        We = 0.081819190842965
        WN = Wa/np.sqrt( 1 - np.power(We,2)*np.power(np.sin(dLat),2))
        WT = np.power(np.tan(dLat), 2)
        WC = (np.power(We,2)/(1-np.power(We,2)))*np.power(np.cos(dLat),2)
        WLA = (dLon - lon0)*np.cos(dLat)
        WM = (Wa*((1 - np.power(We,2)/4 - 3*np.power(We,4)/64 - 5*np.power(We,6)/256)*dLat - (3*np.power(We,2)/8 + 3*np.power(We,4)/32 + 45*np.power(We,6)/1024)*np.sin(2*dLat) + (15*np.power(We,4)/256 + 45*np.power(We,6)/1024)*np.sin(4*dLat) - (35*np.power(We,6)/3072)*np.sin(6*dLat)) )

        Weps = 0.006739496742333
        # Easting
        m_dUTM_X = (FE + k0*WN*(WLA + (1 - WT + WC)*np.power(WLA,3)/6	+ (5 - 18*WT + np.power(WT,2) + 72*WC - 58*Weps)*pow(WLA,5)/120))
        # Northing
        m_dUTM_Y = (FN + k0*WM + k0*WN*np.tan(dLat)*(np.power(WLA,2)/2 + (5 - WT + 9*WC + 4*np.power(WC,2))*np.power(WLA,4)/24 + (61 - 58*WT + np.power(WT,2) + 600*WC - 330*Weps)*np.power(WLA,6)/720))
        # Zone
        m_nUTM_Zone = np.int(np.floor(lon0_f/6)+31)

        return [m_dUTM_X, m_dUTM_Y, 0]

    def update(self):
        # UPDATE POSITION VALUES to print in GUI(LLA & UTM)
        self.label.configure(text="Latitude: {:.2f}\nLongitude: {:.2f}\nAltitude: {:.2f}\n\nUTM X: {:.2f}\nUTM Y: {:.2f}\nUTM Z: {:.2f}\n\nLocal X: {:.2f}\nLocal Y: {:.2f}\nLocal Z: {:.2f}" \
                                  .format(self.lla_data[0], self.lla_data[1], self.lla_data[2], \
                                          self.utm_data[0], self.utm_data[1], self.utm_data[2], \
                                          self.pos_data[0], self.pos_data[1], self.pos_data[2]))        
        
        # Show Path Accumulation
        if self.track_plot_prev is not None:
            self.track_plot_prev.remove()
        self.track_plot_prev = self.a.scatter(self.local_x_arr, self.local_y_arr, color=CFG.PATH_TRACK_COLOR, marker='.', edgecolors='none')

        # Current Position on map (m)
        if self._scat is not None:
            self._scat.remove()
        self._scat = self.a.scatter(self.pos_data[0], self.pos_data[1], color=CFG.CURRENT_POS_COLOR, marker='x', edgecolors='none')

        self.canvas.draw()
        self.label.after(1, self.update)

if __name__ == '__main__':
    
    rospy.init_node('heron_gui_node')

    argumentList = sys.argv[1:]
    options = "ht:"
    long_options = ["Help", "Track_Time"]

    try:
        arguments, values = getopt.getopt(argumentList, options, long_options)
        for currentArgument, currentValue in arguments:
            if currentArgument in ("-h", "--Help"):
                print("-h/--Help: To see available arguments.\n-m/--Mode: 'wp_plot' & 'wp_read'")
            elif currentArgument in ("-t", "--Track_Time"):
                print("Current Track Time", currentValue)
                window = Heron_GUI(currentValue)
                window.mainloop()
            else:
                print("You need a mandatory arrgument -m: wp_plot / wp_read")

    except getopt.error as err:
        print(str(err))

    rospy.spin()

    
