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
    #return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))
    return np.exp(-np.power((x - mu)/sig, 2.)/2)


def tocm(x):
    return x*20

def todegree(x):
    return x*180/3.14159
def todis(x):
    return x*90

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




    b=0
    #start loop here
    #rospy.loginfo(len(MList))
    for b in range(len(MList)):
        #rospy.loginfo(b)
        probsum=0
        temphighx=0
        temphighy=0
        temphighz=0
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
        print("Movement:")
        transmu=MList[r].translation*100
        r1mu=((MList[r].rotation1.z*180)/3.14159)
        r2mu=((MList[r].rotation2.z*180)/3.14159)
        rospy.loginfo(r1mu)
        rospy.loginfo(transmu)
        rospy.loginfo(r2mu)
        rospy.loginfo(MList[r].timeTag)
        print('\n')


        #if r==1:
            #print(grid2[31][34][1])
        #loop1y=-20
        for gy in range(y1):


            loop1y=gy*20+10
            #loop1x=-20
            #rospy.loginfo(gy)
            print(gy)
            for gx in range(x1):
                #rospy.loginfo(gx)

                loop1x=gx*20+10#converting 20cm cells to cm
                #loop1z=-disvalue
                for gz in range(z1):

                    loop1z=gz*disvalue #getting actual angle
                    grid2sum=0

                    #rospy.loginfo(loop1z)
                    #loop2y=-20
                    for gy2 in range(y1):


                        loop2y=gy2*20+10
                        lp=(loop1y-loop2y)
                        #lpsq=lp**2
                        #loop2x=-20
                        for gx2 in range(x1):


                            loop2x=gx2*20+10
                            lp2=(loop1x-loop2x)
                            transx=np.hypot(lp2,lp)

                            tgaus=gaussian(transx,transmu,transsig)


                            ph=(np.arctan2((lp),(lp2))*180)/3.14159
                            #ph2=(np.arctan2((0-lp),(0-lp2))*180)/3.14159
                                #phy=(math.acos((loop2y-loop1y)/transx)*180)/3.14159
                            if ph<0:
                                ph=360+ph

                            r2x=ph-loop1z

                            rgaus=gaussian(r2x,r2mu,r2sig) #c=loop2z+rot1 loop1z=r2+c
                            trgaus=tgaus*rgaus
                            #loop2z=-disvalue

                            for gz2 in range(z1):
                                loop2z=gz2*disvalue
                                #rospy.loginfo(transx)


                                r1x=ph-loop2z
                                #rospy.loginfo(r2x)lmlmoolmom

                                #rospy.loginfo(loop2z)
                                if(grid2[gx2][gy2][gz2]>.01):
                                    grid2sum= grid2sum+ grid2[gx2][gy2][gz2]*trgaus*gaussian(r1x,r1mu,r1sig)
                                    '''if(gx==34 and gy==34 and gz==2 and r==1):
                                        print(grid2sum)
                                        print(gx2)
                                        print(gy2)
                                        print(gz2)
                                        print(grid2[gx2][gy2][gz2])
                                        print(grid2[31][34][1])'''

                                    #if(grid2sum1>grid2sum):
                                        #grid2sum=grid2sum1

                    #vrospy.loginfo(grid2sum)


                    grid[gx][gy][gz]=grid2sum
                    #grid2sum=0
                    #rospy.loginfo(grid2sum)
                    if grid[gx][gy][gz] > temphigh:
                        temphigh=grid[gx][gy][gz]
                        temphighx=gx
                        temphighy=gy
                        temphighz=gz
                        #rospy.loginfo(temphigh)
                        #rospy.loginfo(temphighx)
                        #rospy.loginfo(temphighy)
                        #rospy.loginfo(temphighz)

            #return

                #rospy.loginfo(gy)
        #rospy.loginfo(grid[11][28][2])
        #grid2=grid
        print("Prediciton:")
        rospy.loginfo(temphigh)
        rospy.loginfo(temphighx)
        rospy.loginfo(temphighy)
        rospy.loginfo(temphighz)
        #return
        print('\n')






        rangesig=20
        rangemu=OList[r].range*100
        rangex=0
        bearingsig=90
        bearingmu=(OList[r].bearing.z*180)/3.14159
        #angleconver=(OList[r].bearing.z*180)/3.14159
        #rospy.loginfo(bearingmu)


        #rospy.loginfo(angleconver)
        tagx=0
        tagy=0
        print("Observation:")
        rospy.loginfo(OList[r].tagNum)
        rospy.loginfo(bearingmu)
        rospy.loginfo(OList[r].range*100)
        rospy.loginfo(OList[r].timeTag)
        print('\n')
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
            loop1y=20*y1a+10
            for x1a in range(x1):
                loop1x=20*x1a+10
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
                    #if(cbearing>360 or cbearing <0):





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

        print("Final prediciton:")
        rospy.loginfo(highx)
        rospy.loginfo(highy)
        rospy.loginfo(highz)
        #print('\n')
        rospy.loginfo(highr)
        rospy.loginfo(highpoint)


        sum=np.sum(grid)#normalizes
        grid=grid/sum
        rospy.loginfo(grid[highx][highy][highz])
        print('\n')
        #rospy.loginfo(grid[20][22][2])
        r=r+1
        grid2=copy(grid)
        #rospy.loginfo(grid2[20][22][2])








    return






if __name__ == '__main__':

    Read()


    #rospy.spin()
