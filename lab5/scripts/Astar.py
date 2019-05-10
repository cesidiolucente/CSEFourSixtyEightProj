#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from copy import copy
from copy import deepcopy
import math
import rviz
import rosbag
import std_msgs
import numpy as np
import matplotlib.pyplot as plt
import re

#import numpy
grange=0
class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.gcalc = 0
        self.hcalc = 0
        self.fcalc = 0

    def __eq__(self, other):
        return self.position == other.position


def gridread():
    rospy.init_node('Astar', anonymous=True)

    f = open("/home/first/catkin_ws/src/lab5/world/map.txt","r")
    map1=np.zeros((20,18))
    #mapholder=f.read().split(',')
    mapholder1=f.read()
    mapholder=re.findall(r"[\w']+", mapholder1)
    y=0
    x=0

    #rospy.loginfo(len(mapholder))
    for r in range(len(mapholder)):

        if(mapholder[r]=='1' or mapholder[r]=='0'):
            try:
                map1[y][x]=int(mapholder[r])
            except:
                rospy.loginfo(r)
                rospy.loginfo(mapholder[r])
            x=x+1
        if(x==18):
            x=0
            y=y+1

    '''rospy.loginfo(r)
    rospy.loginfo(x)
    rospy.loginfo(y)
    rospy.loginfo(map1[19][17])'''

    startingx= rospy.get_param('startx')
    startingy=rospy.get_param('starty')
    gx=rospy.get_param('goalx')
    gy=rospy.get_param('goaly')
    #center is 9, 9.8
    gridx, gridy= worldtogrid(startingx, startingy)
    ggridx, ggridy=worldtogrid(gx,gy)
    start=(gridx, gridy)
    end=(ggridx, ggridy)
    #rospy.loginfo(len(map1))
    #rospy.loginfo(len(map1[len(map1)-1]))


    stillopen=[] #list of nodes/places robot hasnt visited
    visitedlist =[] #list of places it has

    startN= Node(None, start) #using a node system to track location

    startN.gcalc = startN.hcalc =startN.fcalc=0 #initialization of start and end node
    endN =Node(None, end)
    endN.gcalc= endN.hcalc =endN.fcalc=0

    stillopen.append(startN)
    clist=[]

    while len(stillopen)>0: #loops until goal is found

        currentID=0
        currentN=stillopen[0] #gets info about current node

        for ID, bestcell in enumerate(stillopen):
            if bestcell.fcalc <currentN.fcalc: #compares f calculation

                currentID=ID

                currentN=bestcell
                t1=0
                for test in clist:
                    if currentN == test:
                        t1=1

                if t1==0:
                    clist.append(currentN)
                    #rospy.loginfo(currentN.position)

        stillopen.pop(currentID) #removes it from open list to closed
        visitedlist.append(currentN)
        #rospy.loginfo(visitedlist[len(visitedlist)-1].position)


        if currentN == endN: #checks if current node is the goal
            path=[]
            current=currentN
            while current is not None: #retraces steps to get a path
                path.append(current.position)
                current = current.parent
            finalpath= path[::-1]
            return finalpath #returns the final path

        neighbors =[] #creates neighbors and calculates the f value of each surrounding grid cell
        for testcell in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            gridcellN = (currentN.position[0] + testcell[0], currentN.position[1] + testcell[1])

            # checks to not go off world map
            if gridcellN[1] > (len(map1) - 1) or gridcellN[1] < 0 or gridcellN[0] > (len(map1[len(map1)-1]) -1) or gridcellN[0] < 0:
                continue

            if (int(map1[gridcellN[1]][gridcellN[0]]) != 0): #makes sure space is traversible
                continue
            if map1[gridcellN[1]+1][gridcellN[0]]!=0 and map1[gridcellN[1]][gridcellN[0]-1] !=0:
                #rospy.loginfo(currentN.position)
                #rospy.loginfo(gridcellN)
                #map1[gridcellN[1]][gridcellN[0]]=1
                continue #checks for thin line by checking the top left and bottem right neighbor

            if map1[gridcellN[1]+1][gridcellN[0]]!=0 and map1[gridcellN[1]][gridcellN[0]+1] !=0:
                #rospy.loginfo(currentN.position)
                #rospy.loginfo(gridcellN)
                #map1[gridcellN[1]][gridcellN[0]]=1
                continue #checks for thin line by checking the top right and bottom left neightbor



            #rospy.loginfo(currentN.position)


            newneighbor= Node(currentN, gridcellN) #if space is ok, adds it as node to list

            neighbors.append(newneighbor)

        for currentneighbor in neighbors:#calculates f value for each neighbor
            visited=0
            shouldadd=0
            for closedcurrentneighbor in visitedlist:  #if neighbor has been visited, then ignores
                if currentneighbor == closedcurrentneighbor:
                    visited=1
                    shouldadd=1
                    continue
            if visited==0:
            #calculates g, h and f value
                currentneighbor.hcalc = ((currentneighbor.position[0] -endN.position[0])**2)+((currentneighbor.position[1]-endN.position[1])**2)

                currentneighbor.gcalc = currentN.gcalc +1
                currentneighbor.fcalc = currentneighbor.gcalc + currentneighbor.hcalc


            for openneighbor in stillopen: #if child is in open list
                if currentneighbor==openneighbor and currentneighbor.gcalc >openneighbor.gcalc:
                    shouldadd=1
                    continue
            if shouldadd==0:
                stillopen.append(currentneighbor)









    rospy.loginfo("Could not find path")
    return



def worldtogrid(worldx,worldy): #calculated world position into grid

    #world is 18, 19.6
    #grid is 18, 20
    #cell size is 1 in x and 1.02 in y

    gridx=round((worldx+8.4)/1)
    gridy=round(abs(worldy-9.4)/1.02)
    print(gridx)
    print(gridy)

    return gridx, gridy

def gridtoworld(gridx,gridy): #calculates grid position into world position

    worldx=gridx-8.5/1
    worldy=(gridy+9.5)/1.02



    return worldx, worldy


def robotcontrol(path):
    #rospy.init_node('control', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    startingx= rospy.get_param('startx')
    startingy=rospy.get_param('starty')


    rospy.Subscriber("/base_scan", LaserScan, callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, callback2)


    for tempgoal in path:
        tempgoalx,tempgoaly=gridtoworld(tempgoal[0],tempgoal[1])
        tg=(tempgoalx,tempgoaly)
        tolerance=.4
        print(tg)
        print(distance(tg))
        while distance(tg) >= tolerance:
            print(currentpose)



             # Linear velocity in the x-axis.
            vel_msg.linear.x = 1.5*distance(tg)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

             # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            #vel_msg.angular.z = math.atan2(tg[1] - currentpose[1],tg[0] - currentpose[0])-currentpose[2]

             # Publishing our vel_msg
            velocity_publisher.publish(vel_msg)
            rospy.sleep(.5)

              # Publish at the desired rate.
             #srate.sleep()

             # Stopping our robot after the movement is over.
    #    vel_msg.linear.x = 0
    #    vel_msg.angular.z = 0
        #velocity_publisher.publish(vel_msg)

             # If we press control + C, the node will stop.
    #rospy.spin()

            #print(1)

            #rospy.loginfo(grange)'''



def distance(goal):
    return math.sqrt(pow((goal[0]-currentpose[0]),2)+pow(goal[1]-currentpose[1],2))

def callback(msg):

    #print(1)
    #rospy.loginfo(msg.ranges[180])
    grange= msg.ranges[45]

    #rospy.loginfo(grange)
    return

def callback2(msg):
    offset=msg.pose.pose.position
    currentpose=(offset.x+rospy.get_param('startx'),offset.y+rospy.get_param('starty'),msg.pose.pose.orientation.z+3.14)

    #print(offset)

    return




if __name__ == '__main__':
    rospy.set_param('goalx', 4.5)
    rospy.set_param('goaly', 9)
    rospy.set_param('startx', -8)
    rospy.set_param('starty', -2)
    global currentpose
    global grange
    currentpose=(rospy.get_param('startx'),rospy.get_param('starty'),0)
    finalpath=gridread()
    print(finalpath)



    robotcontrol(finalpath)

    rospy.loginfo("finished")






    rospy.spin()
