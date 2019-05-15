#!/usr/bin/env python
import rospy, csv, datetime, time, math, pathfinder, random
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

msg = """
Turtle Bot will move automatically
WayPoint test
"""

e = """
Communications Failed
"""
kp = .02
des = 0
angerror = 100.0
oldangerror = 0.0
low_limt = -8.800000
high_limit = 8.800000
coin = 0
end = False
tol = .3
num_wraps = 0
threshold = 30
olddes = 0

map_size = 10
start_loc = [0,0]
goal_loc = [1,9]
sight_radius = 1.0
position = Position()
twist = Twist()


# #waypoints =[[0,0], [0,1], [2,2], [3, -3]]
# unknown = [(0,3),(2,3),(2,4)]
obs = []
unknown = []

obstacles = [(1,1), (4,4), (3,4), (5,0), (5,1), (0,7), (1,7), (2,7), (3,7)]
for i in obstacles:
    rand = random.randint(0,1)
    if rand:
        obs.append(i)
    else:
        unknown.append(i)

pathfinder.send_map_info_for_plot(map_size, obs, unknown)

dist = lambda a, b, c, d : ((abs(c - a)**2 + abs(d - b)**2)**0.5)

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

def pid(command, meas, prev_err, freq, k_p=1, k_i=0, k_d=0):
    '''
    PID Controller Function:

    Inputs:
    command = Desired target value
    meas = Measured value
    prev_err = Error from previous iteration
    freq = Sampling frequency
    k_p = Proportional gain
    k_i = Integrator gain
    k_d = Derivative gain

    Outputs:
    lam_com = Output command signal
    err_prop = Measured error on this iteration
    '''
    # Numeric differentiation by backward finite divided difference method
    ddif = lambda h, f_0, f_1 : (f_0 - f_1) / h
    # Numeric integration by trapezoidal method
    trap = lambda h, f_0, f_1 : h * (f_0 - f_1) / 2
    # Current error
    err_prop = command - meas
    # Integrator error
    err_int = trap(freq, err_prop, prev_err)
    # Derivative error
    err_der = ddif(freq, err_prop, prev_err)
    # Combined lambda signal
    lam_com = (err_prop * k_p) + (err_int * k_i) + (err_der * k_d)
    return lam_com, err_prop

if __name__=="__main__":



    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('quat_2_eul')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    turtlebot3_model = rospy.get_param("model", "burger")

    rospy.Subscriber('/eul', Position, callback_ori, (position))
    rospy.Subscriber('/odom', Odometry, callback_pos, (position))

    time.sleep(1)
    rate = rospy.Rate(100)

    try:

        print msg

        # position = Position()
        # twist = Twist()
        waypoints =  pathfinder.findShortPath(map_size, start_loc, goal_loc, obs)
    	time.sleep(2)

        turning_radius = .001
        step_size = 0.5
        wpm = waypoints

        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        time.sleep(2)
        #set waypoint to first way WayPoint
        wp = wpm[1]
        print wp

        ex = []
        ey = []

        while not rospy.is_shutdown():
            # check current pose
            x = position.linear.x
            y = position.linear.y
            ex.append(x)
            ey.append(y)
            heading = position.angular.yaw
            un = []
            print wpm
            for obst in unknown:
                if dist(x, y, obst[0], obst[1]) < sight_radius:
                    obs.append(obst)
                    unknown.remove(obst)
                    wpm = pathfinder.findShortPath(10, [x, y], goal_loc, obs)
                    time.sleep(0.5)
                    coin = 1
                    wp = wpm[coin]
            # for i in range(len(unknown)):
            #     checking = len(unknown)
            #     if unknown[i][0] < (x+2) or unknown[i][0] > (x-2):
            #         if unknown[i][1] < (y+2) or unknown[i][1] > (y-2):
            #             obs.append(unknown.pop(i))
            #             un.append((i))
            #             wpm =  pathfinder.findShortPath(10,[int(x),int(y)],[1,9],obs)
            #             time.sleep(.5)
            #             coin = 1
            #             wp = wpm[coin]
            #     if len(unknown) < checking:
            #         break
            # if not withen a spec range of waypoint
            # if abs(wp[0] - x) > tol or abs(wp[1] - y) > tol:
            if not ((wp[0] - tol) <= x <= (wp[0] + tol)) or not ((wp[1] - tol) <= y <= (wp[1] + tol)):
                #change heading twords WayPoint
                

                des = math.atan2((wp[1]-y),(wp[0]-x)) * (180/3.14)
                

                if olddes < -threshold and des > threshold: # from -pi to pi (increasing negative)
            		num_wraps = num_wraps - 1
            	elif olddes > threshold and des < -threshold:
            		num_wraps = num_wraps + 1
                
                olddes = des
                des = des + 360 * num_wraps

                angerror = des - position.angular.yaw
                if wp == wpm[1]:
                    twist.linear.x = 0.0
                else:
                    twist.linear.x = 0.2
                cmd, oldangerror = pid(cmd, heading, oldangerror, 1/100.0, .8, 0.0, .15)
                print '[{:1.2f} , {:1.2f}] : Olddes = {:1.2f} Des = {:1.2f} cmd = {:1.2f}'.format(x, y, olddes, des, cmd)



            else:
                if end == False:
                    
                    coin = coin +1
                    if not wpm[coin]:
                        twist.linear.x = 0
                        end = True
                        break
                    wp = wpm[coin]
                    

            # angerror = des - position.angular.yaw
            #if 1 >= error:
            # if wp == wpm[1]:
            #     twist.linear.x = 0.0
            # else:
            #     twist.linear.x = 0.2
            # twist.linear.x = .2
            
            # cmd, oldangerror = pid(cmd, heading, oldangerror, 1/100.0, .8, 0.0, .15)
            # cmd = kp*error
            # print 'CMD = {}'.format(cmd)
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
        # print "wrote"

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