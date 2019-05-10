#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from copy import copy
import math
import rviz
import rosbag
import std_msgs
import numpy as np
#import numpy
def myround(x, base=20):
    return int(base * round(float(x)/base))

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

def Read():
    rospy.init_node('read', anonymous=True)
    bag=rosbag.Bag('grid.bag')
    ph=0
    #mo= msg.Motion
    #ob=msg.Observation
    #might want to use rosbag play which will publishes messages to topcs
    #create standard list then append mesage types to it
    MList=[]
    OList=[]
    for topic1, msg1, t1 in bag.read_messages(topics=['Movements']):
        MList.append(msg1)
        #print msg1
    for topic2, msg2, t2 in bag.read_messages(topics=['Observations']):
        OList.append(msg2)
        #print msg1

    bag.close()

    #rospy.loginfo(len(MList))
    #rospy.loginfo(len(OList))
    startx=12 #what grid location the robot is starting at
    starty=28
    startz=200 #heading degree/
    x1=35
    y1=35
    z1=4 # 10 degree steps but matrix has to be cubed so only us
    #grid= [[[0 for k in range(x1)] for j in range(y1)] for i in range(z1)]
    grid=np.zeros((x1,y1,z1))
    #grid[12][12][11]=20;
    #rospy.loginfo(grid[12][12][11])
    #rospy.loginfo(OList[0].timeTag)
    #rospy.loginfo(MList[0].timeTag)
    prevrot=200 #stores starting heading/rotation, to base the heading that rotation 1 effects
    prevx=12
    prevy=28
    disvalue=90
    r=0
    #grid[12][28][6]=1 #sets initial location in grid to max probability
    grid[12][28][2]=1
    grid2=np.zeros((x1,y1,z1))
    grid2[12][28][2]=1
    #z1=5
    #rospy.loginfo(MList[r].rotation1)
    #rospy.loginfo(MList[r].translation)
    #rospy.loginfo(MList[r].rotation2)
    #mu is mean so actual valuie
    #sig is noise
    #x is calculated value between two point
    probsum=0
    temphighx=0
    temphighy=0
    temphigh=0
    transsig=20
    r1sig=90
    r2sig=90
    loop1z=0
    loop2z=0
    loop1x=0
    loop2x=0
    loop1y=0
    loop2y=0
    transmu=MList[r].translation*100
    r1mu=((MList[r].rotation1.z*180)/3.14159)
    r2mu=((MList[r].rotation2.z*180)/3.14159)
    #rospy.loginfo(r1mu)
    #rospy.loginfo(r2mu)


    x1a=11
    x2a=12
    y1a=28
    y2a=28
    z1a=8
    z2a=8
    grid2sum=0
    lp=(y1a*20-y2a*20)
    lpsq=lp**2
    transx=math.sqrt((x1a*20-x2a*20)**2+lpsq)
    tgaus=gaussian(transx,transmu,transsig)
    #rospy.loginfo(tgaus)
    #rospy.loginfo(transx)

    if x1a==x2a:
        if lp>0:
            ph=90
        elif lp==0:
            ph=0
        else:
            ph=270
    elif lp==0:
        ph=z1a*36

    else:
        ph=(math.atan((lp/(x1a-x2a))*180)/3.14159)
    #rospy.loginfo(ph)
    rx2=z1a*90-ph
    rx1=ph-z2a*90
    #rospy.loginfo(rx1)
    #rospy.loginfo(rx2)
    rgaus=gaussian(rx1,r1mu,r1sig)
    #rospy.loginfo(rgaus)
    r2gaus=gaussian(rx2,r2mu,r2sig)
    #rospy.loginfo(r2gaus)
    grid2sum= grid2sum +grid2[12][28][2]*tgaus*rgaus*r2gaus


    #rospy.loginfo(grid2sum)
    #return




    rangesig=20
    rangemu=OList[r].range*100
    rangex=0
    bearingsig=90
    bearingmu=(OList[r].bearing.z*180)/3.14159
    angleconver=(OList[r].bearing.z*180)/3.14159
    #rospy.loginfo(bearingmu)


    #rospy.loginfo(angleconver)
    tagx=0
    tagy=0
    #rospy.loginfo(OList[r].tagNum)
    if(OList[r].tagNum==0):
        tagx=125
        tagy=525
    elif(OList[r].tagNum==1):
        tagx=125
        tagy=325
    elif(OList[r].tagNum==2):
        tagx=125
        tagy=125
    elif(OList[r].tagNum==3):
        tagx=425
        tagy=125
    elif(OList[r].tagNum==4):
        tagx=425
        tagy=325
    elif(OList[r].tagNum==5):
        tagx=425
        tagy=525
    #rospy.loginfo(OList[r].tagNum)
    #rospy.loginfo(tagy)
    #rospy.loginfo(OList[r].range*100)
    #fx1=myround(tagx-OList[r].range*100*math.cos(tagheading*3.14159/180))/20
    #fy1=myround(tagy-OList[r].range*100*math.sin(tagheading*3.14159/180))/20
    highpoint=0
    highx=0
    highy=0
    highz=0

    for y1a in range(y1):
        loop1y=20*y1a
        for x1a in range(x1):
            loop1x=20*x1a
            for z1a in range(z1):
                loop1z=90*z1a


                t1=tagy-loop1y
                t2=tagx-loop1x

                crange=np.hypot(t1,t2)




                tanang=(np.arctan2((t1),(t2))*180/3.14159)
                '''if(x1a==2 and y1a==26 and z1a==2):
                    rospy.loginfo(tanang)
                    rospy.loginfo(t1)
                    rospy.loginfo(t2)
                if(x1a==10 and y1a==28 and z1a==2 ):
                    rospy.loginfo(tanang)
                    rospy.loginfo(t1)
                    rospy.loginfo(t2)'''


                if (tanang<0):
                    tanang=tanang+360
                cbearing=tanang-loop1z




                #if cbearing<0:
                    #cbearing=cbearing+360
                #grid[y1a][x1a][z1a]=grid[y1a][x1a][z1a]*gaussian(crange,rangemu,rangesig)*gaussian(cbearing,bearingmu,bearingsig)
                grid[x1a][y1a][z1a]=grid[x1a][y1a][z1a]*gaussian(crange,rangemu,rangesig)*gaussian(cbearing,bearingmu,bearingsig)

                if(grid[x1a][y1a][z1a]>highpoint):
                    highx=x1a
                    highc=cbearing+loop1z
                    highp=loop1z
                    highy=y1a
                    highz=z1a
                    highr=tanang
                    highpoint=grid[x1a][y1a][z1a]



                '''for y2a in range(y1):
                    loop2y=20*y2a
                    for x2a in range(x1):
                        loop2x=20*x2a

                        for z2a in range(z1):
                            loop2x=90*z2a

                            '''

    rospy.loginfo(highx)
    rospy.loginfo(highy)
    rospy.loginfo(highz)
    #rospy.loginfo(highr)
    rospy.loginfo(highpoint)
    #rospy.loginfo(highc)
    #rospy.loginfo(highp)
    rospy.loginfo(grid[11][28][2])
    rospy.loginfo(grid[10][28][2])
    #rospy.loginfo(grid[11][27][2])
    #rospy.loginfo(grid[10][27][2])


    #right now fx1 and fy1 is location based off of Observation
    #and finalx adn finaly are based of movmement
    #do gaussian for both to get the new prevx and prevy to calculate the next movmeent













    return






if __name__ == '__main__':

    Read()


    #rospy.spin()
