#! usr#!/usr/bin/env/python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf.transformations
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation

global state
global mapX
global mapY
mapX = []
mapY = []
state = 0
fig, ax = plt.subplots()

ln, = plt.plot([], [], 'ro')
 
#Turtlebot Shutdown   
def shutdown():
    rospy.loginfo("Stop Turtlebot")
    pub.publish(Twist())
    rospy.sleep(1)

    
class Turtlebot_Map:
    global mapX
    global mapY
    def __init__(self):
        self.position = None
        self.lidar_points = None
        self.prev = None
        
    #WALL TRACING FUNCTION    
    def lidar(self, msg):
        self.lidar_points = msg.ranges
        print("=========================")
    
        print('s1 [0]')
        print(msg.ranges[0])
    
        print('s2 [90]')
        print(msg.ranges[90])

        print('s2 [180]')
        print(msg.ranges[180])
        
        print('s2 [270]')
        print(msg.ranges[270])
    
        print('s2 [359]')
        print(msg.ranges[359])
    
        global state
    
        #Obstacle Scanner
        wall = False  
        obs_front = False 
        obs_right = False
        obs_left = False
        
        #WALL FINDER SWEEP FRONT       
        for w in range (0,15):
            if (msg.ranges[w] > 0.8 and msg.ranges[w] != 0) or (msg.ranges[359 - w] > 0.8 and msg.ranges[359-w] != 0):    
                continue
            else:
                wall = True 
        
        #OBSTACLE FRONT SCANNER
        for i in range (0,30):
            if msg.ranges[i] > 0.30 and msg.ranges[i] != 0:
                continue
            else:
                obs_front = True
                
        for j in range (330,360):
            if msg.ranges[j] > 0.30 and msg.ranges[j] != 0:
                continue
            else:
                obs_front = True  
        
        #OBSTACLE RIGHT SCANNER 
        for k in range (270,330):
            if msg.ranges[k] > 0.30 and msg.ranges[k] != 0:
                continue
            else:
                obs_right = True  
        #OBSTACLE LEFT SCANNER 
        for l in range (30,90):
            if msg.ranges[l] > 0.30 and msg.ranges[l] != 0:
                continue
            else:
                obs_left = True 
        
        #WALL FINDER STATE
        if state == 0:
            
            if obs_front == True:
                state = 1
            elif obs_right == True and obs_front == True:
                state = 1    
            elif obs_left == True:
                state = 2
            elif obs_left == True and obs_front == True:
                state = 2
            #Turtle bot will turn clockwise to find wall
            elif wall == False:
                move.linear.x = 0.0
                move.angular.z = -1.2
                print('Finding Wall')
            elif wall == True:
                state = 3
        
        #RIGHT FOLLOWING STATE
        elif state == 1:
            
            if obs_front == True and obs_right == True:
                move.linear.x = 0.0
                move.angular.z = 1.2
                print("Obstacle in Front and in Right, Turning Left")
            elif obs_front == False and obs_right == False:
                move.linear.x = 0.1
                print("Finding Wall, Turning Right")
            elif obs_front == True and obs_right == False:
                move.linear.x = 0
                move.angular.z = 1.2
                print("Obstacle in Front, Turning Left")
            elif obs_front == False and obs_right == True:
                move.linear.x = 0.22
                move.angular.z = 0
                print("Tracing Wall on Right, Moving Forward")
        
        #LEFT FOLLOWING STATE
        elif state == 2:
            if obs_front == True and obs_left == True:
                move.linear.x = 0.0
                move.angular.z = -1.2
                print("Obstacle in Front and in Left, Turning Right")
            elif obs_front == False and obs_left == False:
                move.linear.x = 0.1
                move.angular.z = 1.2
                print("Finding Wall, Turning Left")
            elif obs_front == True and obs_left == False:
                move.linear.x = 0
                move.angular.z = -1.2
                print("Obstacle in Front, Turning Right")
            elif obs_front == False and obs_left == True:
                move.linear.x = 0.22
                move.angular.z = 0
                print("Tracing Wall on Left, Moving Forward")
        
        #IF WALL IS TRUE THE TURTLEBOT WILL MOVE FORWARD THEN CHANGE STATE
        elif state == 3:
            if wall == True:
                move.linear.x = 0.22
                move.angular.z = 0
                print("Moving Forward to the nearest Wall")
            
                if obs_right == True:
                    state = 1
                elif obs_left == True:
                    state = 2
                elif obs_front == True:
                    state = 4
        
        #WALL IS TRUE AND NO OTHER OBSTACLE SCAN STATE
        elif state == 4:
            state = 4.1
            if obs_front == True and state == 4.1:
                move.linear.x = 0
                move.angular.z = 1.2
                print("Turning Left")
                state = 1
        
        pub.publish(move) 

    #MAPPING FUNCTION
    def odom(self, msg):
        quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        #print(self.position)
        #print("yaw", self.yaw)
        
        self.position = msg.pose.pose.position
        
        #PREV POSITION CONDITION
        if self.prev == None:
            self.prev = self.position    
        
        x_ = (self.position.x - self.prev.x)**2
        y_ = (self.position.y - self.prev.y)**2
        z_ = (self.position.y - self.prev.y)**2
        d = math.sqrt(x_ + y_ + z_)#DISPLACEMENT
        
        #CALL APPEND FUNCTION ONLY IF DISPLACEMENT CONDITION IS SATISFIED
        if d > 0.5:
            self.store()
            self.prev = self.position #POSITION UPDATE
    
    #APPEND FUNCTION    
    def store(self):
        for i in range (360):
            x_coor = self.position.x + self.lidar_points[i]*math.cos(math.radians(i) + self.yaw)       
            y_coor = -1*(self.position.y + self.lidar_points[i]*math.sin(math.radians(i) + self.yaw))
            
            #APPEND ONLY WHEN MOVING FORWARD
            if move.angular.z == 0:
                mapX.append(x_coor)
                mapY.append(y_coor)                

#Bonus Reference: https://matplotlib.org/3.1.1/api/animation_api.html 

#MAP PLOT COORDINATES
def coor():
    plt.xlim(-3,3)
    plt.ylim(-3,3)
    return ln,

#MAP POINTS UPDATE
def bonus(frame):
    ln.set_data(mapX, mapY)
    return ln,

            
            
rospy.init_node("mapping")
rospy.loginfo("To stop the Turtlebot CTRL+C")
rospy.on_shutdown(shutdown)
 
Turtlebot = Turtlebot_Map()



sub_lidar = rospy.Subscriber ('/scan', LaserScan, Turtlebot.lidar)
sub_odom = rospy.Subscriber ('/odom', Odometry, Turtlebot.odom)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
move = Twist()
ani_map = FuncAnimation(fig, bonus, init_func=coor)
plt.show()
rospy.spin()

