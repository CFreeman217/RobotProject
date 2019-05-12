#!/usr/bin/env python
import rospy
import pathfinder
import Dij0
import numpy
import csv,datetime
import dubins
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time
import math
from geometry_msgs.msg import Twist
from beginner_tutorials.msg import Position
from nav_msgs.msg import Odometry
import sys, select, os
if os.name == 'nt': # if windows
  import msvcrt
else:
  import tty, termios
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection

def callback_pos(data, data_out):
	data_out.linear.x = data.pose.pose.position.x
	data_out.linear.y = data.pose.pose.position.y
	data_out.linear.z = data.pose.pose.position.z

def callback_ori(data, data_out):

	data_out.angular.roll  = data.angular.roll
	data_out.angular.pitch = data.angular.pitch
	data_out.angular.yaw   = data.angular.yaw

def get_filename(prefix, suffix, base_path):
    '''
    Gets a unique file name in the base path.
    
    Appends date and time information to file name and adds a number
    if the file name is stil not unique.

    prefix = Homework assignment name

    suffix = Extension

    base_path = Location of log file
    '''
    # Set base filename for compare
    fileNameBase = base_path + prefix + "_" + datetime.datetime.now().strftime("%b_%d_%H_%M")
    # Set base for numbering system if filename exists
    num = 1
    # Generate complete filename to check existence
    fileName = fileNameBase + suffix
    # Find a unique filename
    while os.path.isfile(fileName):
        # if the filename is not unique, add a number to the end of it
        fileName = fileNameBase + "_" + str(num) + suffix
        # increments the number in case the filename is still not unique
        num = num + 1
    return fileName

def draw_square(coords, side=1.0, sq_color='k'):
    rect = [mpatches.Rectangle((coords[0]-side*0.5, coords[1]-side*0.5), side, side)]
    pc = PatchCollection(rect, facecolor=sq_color, edgecolor='None')
    ax.add_collection(pc)


msg = """
Turtle Bot will move automatically
WayPoint test
"""

e = """
Communications Failed
"""
kp = .02
des = 0
error = 0
low_limt = -8.800000
high_limit = 8.800000
coin = 0
end = False
tol = .3
num_wraps = 0
threshold = 30
olddes = 0


#waypoints =[[0,0], [0,1], [2,2], [3, -3]]


if __name__=="__main__":

    position = Position()
    twist = Twist()

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('Move')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    turtlebot3_model = rospy.get_param("model", "burger")

    rospy.Subscriber('/eul', Position, callback_ori, (position))
    rospy.Subscriber('/odom', Odometry, callback_pos, (position))

    time.sleep(1)
    rate = rospy.Rate(100)

    try:

        print msg

        unknown = [(0,3),(2,3),(2,4)]

        obs = [(1,1), (4,4), (3,4), (5,0), (5,1), (0,7), (1,7), (2,7), (3,7)]
        #waypoints =  RRT_hw.RRT([10,10],[0,0],[1,9],[[1,1], [4,4], [3,4], [5,0], [5,1], [0,7], [1,7], [2,7], [3,7]])
        pathfinder.send_map_info_for_plot(10, obs, unknown)
        waypoints =  Dij0.findShortPath(10,[0,0],[1,9],obs)
	print "not rrt"
	time.sleep(2)

        print waypoints
        turning_radius = .001
        step_size = 0.5
        wpm = waypoints
        print 'here'









        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        time.sleep(2)


        #set waypoint to first way WayPoint
        wp = wpm[coin]
        print wpm

        ex = []
        ey = []

        while not rospy.is_shutdown():




            # check current pose
            x = position.linear.x
            y = position.linear.y
            ex.append(x)
            ey.append(y)

            print "heelo"
            un = []
            for i in range(len(unknown)):
                checking = len(unknown)
                print "heelo"
                if unknown[i][0] < (x+2) or unknown[i][0] > (x-2):
                    print "heelo"
                    if unknown[i][1] < (y+2) or unknown[i][1] > (y-2):
                        print "heelo"
                        obs.append(unknown.pop(i))


                        print 'heelo'
                        un.append((i))
                        print 'heelo'



                        wpm =  Dij0.findShortPath(10,[int(x),int(y)],[1,9],obs)
                        time.sleep(.5)
                        coin = 1
                        wp = wpm[coin]
                if len(unknown) < checking:
                    break


            print "heeloooo"
            print len(un)
            #for i in range(len(un)):
            #    print "heeloooo"
                #unknown.remove(unknown[un[i]])




            # if not withen a spec range of waypoint
            if not ((wp[0] - tol) <= x <= (wp[0] + tol)) or not ((wp[1] - tol) <= y <= (wp[1] + tol)):
                #change heading twords WayPoint
                des = math.atan2((wp[1]-y),(wp[0]-x)) * (180/3.14)
                print des

                if olddes < -threshold and des > threshold: # from -pi to pi (increasing negative)
            		num_wraps = num_wraps - 1
            	elif olddes > threshold and des < -threshold:
            		num_wraps = num_wraps + 1

                olddes = des
                des = des + 360 * num_wraps



            else:
                if end == False:

                    coin = coin +1
                    print coin
                    wp = wpm[coin]
                    if not wpm[coin]:
                        twist.linear.x = 0
                        end = True
                        break


            error = des - position.angular.yaw



            #if 1 >= error:
            twist.linear.x = .2

            cmd = kp*error

            if cmd > high_limit:
                cmd = high_limit
            if cmd < low_limt:
                cdm = low_limt


            twist.angular.z = cmd
            pub.publish(twist)
            rate.sleep()

    except:

        print e

    finally:
        filename = get_filename('final_proj','.csv','./log_files/')
        new_list = zip(ex, ey)
        with open(filename, 'wb+') as csvfile:
             filewriter = csv.writer(csvfile)
             filewriter.writerows(new_list)
        print "wrote"

        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

pos_x, pos_y = np.loadtxt(filename,
                        delimiter=',',
                        skiprows=1,
                        unpack=True)
fig, ax = plt.subplots()
plt.plot(pos_x, pos_y, label='Turtlebot Position')
for point in obs:
    draw_square(point)
plt.title('Turtlebot Adaptive Real Time Path Planning')
plt.xlabel('X-Position (m)')
plt.ylabel('Y-Position (m)')
plt.grid()
plt.savefig(filename[:-4] + 'figure.png', bbox_inches='tight')
plt.show()