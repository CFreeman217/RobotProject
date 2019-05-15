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

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = .75

old_ang = 0.0

low_limt = -8.800000
high_limit = 8.800000

tol = .3
num_wraps = 0
threshold = 30

position = Position()
twist = Twist()
odom = Odometry()

map_size = 15
start_loc = [1.0, 1.0]
goal_loc = [7.0, 13.0]
sight_radius = 1.0


known_obs = []
unknown_obs = []

# all_obstacles = [(1,1), (4,4), (3,4), (5,0), (5,1), (0,7), (1,7), (2,7), (3,7)]
all_obstacles = [(2,3),
             (2,4),
             (2,5),
             (0,5),
             (1,5),
             (2,5),
             (3,5),
             (4,5),
             (5,5),
             (5,4),
             (5,3),
             (8,2),
             (9,2),
             (10,2),
             (11,2),
             (12,2),
             (13,2),
             (8,3),
             (8,4),
             (8,5),
             (8,6),
             (8,7),
             (8,8),
             (8,9),
             (8,7),
             (2,7),
             (3,7),
             (4,7),
             (5,7),
             (6,7),
             (7,7),
             (9,6),
             (10,6),
             (11,6),
             (12,6),
             (13,6),
             (14,6),
             (15,8),
             (2,8),
             (2,9),
             (2,10),
             (2,11),
             (2,12),
             (2,13),
             (5,9),
             (5,10),
             (5,11),
             (5,12),
             (5,13),
             (5,14),
             (5,15),
             (6,12),
             (7,12),
             (8,12),
             (9,12),
             (10,12),
             (11,12),
             (12,8),
             (12,9),
             (12,10),
             (12,11),
             (12,12)]

for i in all_obstacles:
    rand = random.randint(0,1)
    if rand:
        known_obs.append(i)
    else:
        unknown_obs.append(i)

pathfinder.send_map_info_for_plot(map_size, known_obs, unknown_obs)

dist = lambda a, b, c, d : ((abs(c - a)**2 + abs(d - b)**2)**0.5)
rads = lambda x : math.pi * x / 180.0
degs = lambda x : 180.0 * x / math.pi

# callback function for 3DOF linear position
def callback_pos(data):

	# the following line allows this function to access the variable
	# called position which exists in the global namespace, without
	# this statement using the global keyword, we will get an error
	# that the local variable "position" has been used prior to
	# being declared
	global position

	position.linear.x = data.pose.pose.position.x
	position.linear.y = data.pose.pose.position.y
	position.linear.z = data.pose.pose.position.z

# callback function for 3DOF rotation position
def callback_ori(data):

	global position

	position.angular.roll  = data.angular.roll
	position.angular.pitch = data.angular.pitch
	position.angular.yaw   = data.angular.yaw


def draw_square(coords, side=1.0, sq_color='k'):
    rect = [mpatches.Rectangle((coords[0]-side*0.5, coords[1]-side*0.5), side, side)]
    pc = PatchCollection(rect, facecolor=sq_color, edgecolor='None')
    ax.add_collection(pc)

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

def absolute_angle(in_angle):
    ''' 
    Gets absolute angle
    Converts when input angle is more than 360 degrees
    '''
    count = int(in_angle) / 360

    out_angle = in_angle - count*360.0
    return out_angle

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

    rospy.init_node('finalDriver', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/eul', Position, callback_ori)
    rospy.Subscriber('/odom', Odometry, callback_pos)
    turtlebot3_model = rospy.get_param("model", "burger")
    rate = rospy.Rate(100)
    # Start with zero velocity
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    # pub.publish(twist)
    
    time.sleep(2.0)
    print msg
    fileName = get_filename('finalProj','.csv',"./log_files/")
    myData = ['pos_x, pos_y']
    with open(fileName, 'a') as myFile:
        writer = csv.writer(myFile)
        writer.writerow(myData)
    x = position.linear.x
    y = position.linear.y
    
    # heading = position.angular.yaw
    waypoints = pathfinder.findShortPath(map_size, start_loc, goal_loc, known_obs)
    ang_err = 100
    wpt = 0
    while True:
        x = position.linear.x
        y = position.linear.y
        heading = absolute_angle(position.angular.yaw)
        des_x = waypoints[wpt][0]
        des_y = waypoints[wpt][1]
        
        err_x = des_x - x
        err_y = des_y - y
        des_heading = degs(math.atan2(err_y, err_x))
        if des_heading < 0.0:
            des_heading += 360.0
        dist_err = math.sqrt(err_x**2 + err_y**2)
        print '({},{}) @ {} Desired Heading: {:1.2f}'.format(x, y, heading, des_heading)
        if abs(dist_err) < tol:
            ### Cycle to the next waypoint'''
            print 'waypoint {} reached'.format(wpt)
            wpt += 1
            if wpt == len(waypoints):
                break
            ### When we reach the end of the waypoint list, stop and end the program '''
        else:
            for obs in unknown_obs:
                if dist(x, y, obs[0], obs[1]) < sight_radius:
                    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
                    pub.publish(twist)
                    print 'New Obstacle Found at {}'.format(obs)
                    known_obs.append(obs)
                    unknown_obs.remove(obs)
                    waypoints = pathfinder.findShortPath(map_size, [int(x), int(y)], goal_loc, known_obs)
                    print waypoints
                    wpt = 0
            
            if old_ang < -threshold and heading > threshold:
                num_wraps = num_wraps - 1
            elif old_ang > threshold and heading < -threshold:
                num_wraps = num_wraps + 1
            old_ang = heading
            heading = heading + 360 * num_wraps

            if des_heading < threshold or (360 - des_heading) < threshold:
                difference = heading - des_heading
                if difference > 4 * threshold:
                    des_heading += 360.0     
             
                


            ang_err = heading - des_heading
            # print 'Heading = {:1.2f}, Desired: {:1.2f}'.format(heading, des_heading)
            if abs(ang_err) > threshold:
                twist.linear.x = 0.0
            else:
                twist.linear.x = BURGER_MAX_LIN_VEL
            
            turn_cmd, ang_err = pid(rads(des_heading), rads(heading), ang_err, 1.0, 0.8, 0.0, 0.15)

            if turn_cmd > BURGER_MAX_ANG_VEL:
                turn_cmd = BURGER_MAX_ANG_VEL
            if turn_cmd < BURGER_MAX_ANG_VEL:
                turn_cmd = -BURGER_MAX_ANG_VEL
            # print turn_cmd
            twist.angular.z = turn_cmd
            pub.publish(twist)
            rate.sleep()

            myData = [x, y]
            with open(fileName, 'a') as myFile:
                writer = csv.writer(myFile)
                writer.writerow(myData)

    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub.publish(twist)


    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    pos_x, pos_y = np.loadtxt(fileName,
                            delimiter = ',',
                            skiprows = 1,
                            usecols=(0,1),
                            unpack=True)
    fig, ax = plt.subplots()
    plt.plot(pos_x, pos_y, label="Recorded Position", linewidth=3)
    plt.title("Hidden Obstacle Driver Position Data")
    plt.xlabel("X-Position (m)")
    plt.ylabel("Y-Position (m)")
    plt.axis([-1.0, map_size+1.0, -1.0, map_size+1.0],'square')
    for obs in all_obstacles:
        draw_square(obs,0.8)
    plt.grid()
    # plt.legend(loc='lower right')
    plt.savefig("./log_files/" + "final_proj_paths.png", bbox_inches='tight')
    plt.show()