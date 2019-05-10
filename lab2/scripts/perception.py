#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from std_msgs.msg import Float64

from copy import copy
import math
import rviz
import random
global pose
global grange1
global grange2



def callback(msg):
    callback.counter+=1 #makes sure that the code doesnt run more often then necessary, reducing weightn
    if(callback.counter%2==0):

        xpose=0
        ypose=0
        rang=[]
        #rospy.loginfo(len(msg.ranges))
        for x in range(0,360): #loop that gets all important range data
            #rospy.loginfo(msg.ranges[x])
            xpose= (msg.ranges[x]*math.cos((x/2)*3.14/180))
            ypose= (msg.ranges[x]*math.sin((x/2)*3.14/180))
            if(msg.ranges[x]<1.5):
                rang.append([xpose,ypose,0])






        vel_msg = Twist() #assigning some variables
        rvis= Marker()
        p=Point()
        temp=Point()

        p1=Point()

        ph=copy(len(rang))
        #rospy.loginfo(ph)
        rvis.header.frame_id= 'base_link' #rvis setup

        rvis.type=Marker.LINE_STRIP
        rvis.scale.x = 0.45
        rvis.scale.y = 0.45
        rvis.scale.z = 0.45
        rvis.color.r = 0.0
        rvis.color.g = 0.5
        rvis.color.b = 0.5
        rvis.color.a = 1.0

        while(len(rang)>ph/3): #while there are still a third or more sensor points, process them



            binner=.2 #distance of point to decide if inlier or outlier
            lineamount=5 #amount of lines that are created off of one set of points
            gline1=Point()
            gline2=Point()
            ginlier=0
            newrang=rang
            linecounter=0


            for y in range(0,lineamount): #finds 20 lines created from random points and chooses the best
                inliers=[]
                outliers=[]

                angle1=random.randint(0,len(rang)-1) #chooses 2 random points to drawe the line from
                angle2=random.randint(0,len(rang)-1)

                p.x=rang[angle1][0]
                p.y=rang[angle1][1]
                p.z=0


                p1.x=rang[angle2][0]
                p1.y=rang[angle2][1]
                p1.z=0
                trang=copy(rang)
                z=0
                r=0
                try: #makes sure the math is possible, if points are too close can make slope math impossible
                    slope= (p1.x-p.x)/(p1.y-p.y)
                    intercept= p1.y -p1.x*slope

                except:#  if math is not possible, skips the line combination
                    slope=999
                    intercept=0
                    trang=[]




                while (len(trang)>0): #checks through all relivant points to see if binned inlier or outlier


                    nx,ny=getnormal(slope, intercept, trang[0][0],trang[0][1]) #gets point on line as close to chosen point

                    d=math.sqrt((trang[0][0]- nx)**2 + (trang[0][1]- ny)**2) #gets distance of normal from line
                    #abs(p.x-trang[z][0])+abs(p.y-trang[z][1])+abs(p1.x-trang[z][0])+abs(p.y-trang[z][1])
                    #math.sqrt((abs(p.x-trang[z][0])+abs(p.y-trang[z][1]))**2+ (abs(p1.x-trang[z][0])+abs(p.y-trang[z][1])**2))
                    #rospy.loginfo(d)
                    #rospy.loginfo(trang[z][1])




                    if (d>binner): #decides if point is inlier or outlier
                        outliers.append([copy(trang[z][0]),copy(trang[z][1]),0])

                    else:
                        inliers.append([copy(trang[z][0]),copy(trang[z][1]),0])
                        #rospy.loginfo(inliers[0][1])

                    trang.pop() #delets that point from the temp stack
                #rospy.loginfo(len(inliers))
                if (len(inliers)>ginlier): #chooses the line with the most inliers
                    gline1=copy(p) #saves the 2 points
                    gline2=copy(p1)
                    newrang=copy(outliers) #sets total points to the outliers, getting rid of the inliers
                    ginliner=copy(len(inliers))




            rang=copy(newrang)#sets point info to get rid of inliers


            rvis.points=[] #clears then publishes rvis points

            p=gline1
            #rospy.loginfo(p)

            rvis.points.append(p)

            p1=gline2


            rvis.points.append(p1)

            slopeph=Float64()
            slopeph=slope
            slopepub=rospy.Publisher('slope', Float64 ,queue_size=10)

            slopepub.publish(slope)


            rvispub= rospy.Publisher('rvis', Marker, queue_size=10)
            rvispub.publish(rvis)


        #rospy.loginfo(grange)

    return
callback.counter=0

def getpose(msg):
    global pose
    #pose = odom_data.pose.pose.position.x



    return

def getnormal(slope, intercept,x0,y0): #uses math to get normal point on the line
    x=(x0+slope*y0-slope*intercept)/(1+slope**2)
    y=((slope*x0 +(slope**2)*y0-(slope**2)*intercept)/(1+slope**2))+intercept
    return x, y



if __name__ == '__main__':

    rospy.init_node('perception', anonymous=True)

    rospy.Subscriber("/base_scan", LaserScan, callback)


    rospy.spin()
