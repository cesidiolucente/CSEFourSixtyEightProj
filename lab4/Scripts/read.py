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
#import numpy


def gaussian(x, mu, sig): #gaussian function
    #return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))
    return np.exp(-np.power((x - mu)/sig, 2.)/2)



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

    x1=35
    y1=35
    z1=8# 10 degree steps but matrix has to be cubed so only us
    #grid= [[[0 for k in range(x1)] for j in range(y1)] for i in range(z1)]
    grid=np.zeros((x1,y1,z1))


    disvalue=45
    r=0
    #grid[12][28][6]=1 #sets initial location in grid to max probability
    grid[12][28][4]=1
    grid2=np.zeros((x1,y1,z1))
    grid2[12][28][4]=1
    xlist=[12,12]
    ylist=[28,28]



    #try:
    #    f= open("trajectory.txt", "x") #creates file
    #except:
    f = open("trajectory.txt","w")
    f.write("Highest probability")

    b=0
    #start loop here
    #rospy.loginfo(len(MList))
    for b in range(len(MList)):

        grid2sum=0 #initializes values
        temphighx=0
        temphighy=0
        temphighz=0
        temphigh=0
        transsig=10
        r1sig=disvalue/2
        r2sig=disvalue/2
        loop1z=0
        loop2z=0
        loop1x=0
        loop2x=0
        loop1y=0
        loop2y=0
        #print("Movement:",MList[r].timeTag)
        transmu=MList[r].translation*100
        r1mu=((MList[r].rotation1.z*180)/3.14159)
        r2mu=((MList[r].rotation2.z*180)/3.14159)
        '''rospy.loginfo(r1mu)
        rospy.loginfo(transmu)
        rospy.loginfo(r2mu)'''
        #rospy.loginfo(MList[r].timeTag)
        #print('\n')



        for gy in range(y1):
            #print(gy)
            #loop1y=(gy*20)+10
            for gx in range(x1):
                #loop1x=(gx*20)+10#converting 20cm cells to cm
                for gz in range(z1):
                    #loop1z=gz*disvalue #getting actual angle
                    for gy2 in range(y1):
                        #loop2y=(gy2*20)+10
                        #lp=(loop1y-loop2y)
                        for gx2 in range(x1):
                            #loop2x=(gx2*20)+10
                            #lp2=(loop1x-loop2x)
                            for gz2 in range(z1):

                                if(grid2[gx2][gy2][gz2]>.01):
                                    loop1y=(gy*20)+10
                                    loop1x=(gx*20)+10#converting 20cm cells to cm
                                    loop1z=gz*disvalue #getting actual angle'''
                                    loop2y=(gy2*20)+10
                                    lp=(loop1y-loop2y)
                                    loop2x=(gx2*20)+10
                                    lp2=(loop1x-loop2x)
                                    transx=np.hypot(lp2,lp)
                                    ph=(np.arctan2((lp),(lp2))*180)/3.14159
                                    if ph<0:
                                        ph=360+ph

                                    r2x=loop1z-ph
                                    loop2z=gz2*disvalue
                                    #rospy.loginfo(transx)
                                    r1x=ph-loop2z
                                    grid2sum= grid2sum+ grid2[gx2][gy2][gz2]*gaussian(transx,transmu,transsig)*gaussian(r2x,r2mu,r2sig)*gaussian(r1x,r1mu,r1sig)



                    grid[gx][gy][gz]=copy(grid2sum) #sum of probabilities placed into grid

                    grid2sum=0
                    #rospy.loginfo(grid2sum)







        rangesig=10 #range noise 8cm
        rangemu=OList[r].range*100 #defines mu
        rangex=0
        bearingsig=disvalue/2 #discrartiation value /2
        bearingmu=(OList[r].bearing.z*180)/3.14159
        #angleconver=(OList[r].bearing.z*180)/3.14159
        #rospy.loginfo(bearingmu)


        #rospy.loginfo(angleconver)
        tagx=0
        tagy=0
        '''print("Observation:",OList[r].timeTag)
        rospy.loginfo(OList[r].tagNum)
        rospy.loginfo(bearingmu)
        rospy.loginfo(OList[r].range*100)
        #rospy.loginfo(OList[r].timeTag)
        print('\n')'''
        if(OList[r].tagNum==0): #defines values for observation step
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

        highpoint=0
        hp2=0
        hx2=0
        hy2=0
        hz2=0
        highx=0
        highy=0
        highz=0

        for y1a in range(y1): #loops goes along grid and calculates probabilities with respect to the location and heading compared to the landmark
            loop1y=(20*y1a)+10
            for x1a in range(x1):
                loop1x=(20*x1a)+10
                for z1a in range(z1):
                    loop1z=disvalue*z1a


                    t1=tagy-loop1y
                    t2=tagx-loop1x

                    crange=np.hypot(t1,t2)
                    tanang=(np.arctan2((t1),(t2))*180/3.14159)

                    if (tanang<0):
                        tanang=tanang+360
                    cbearing=tanang-loop1z

                    grid[x1a][y1a][z1a]=grid[x1a][y1a][z1a]*gaussian(crange,rangemu,rangesig)*gaussian(cbearing,bearingmu,bearingsig)

                    if(grid[x1a][y1a][z1a]>highpoint):

                        highx=copy(x1a)

                        highy=copy(y1a)
                        highz=copy(z1a)

                        highpoint=copy(grid[x1a][y1a][z1a])

        print("Final prediciton:",b+1)
        rospy.loginfo(highx)
        rospy.loginfo(highy)
        rospy.loginfo(highz)
        #print('\n')
        #rospy.loginfo(highr)
        rospy.loginfo(highpoint)



        xlist.append(highx)
        ylist.append(highy)
        #rospy.loginfo(len(xlist))
        #rospy.loginfo(grid[hx2][hy2][hz2])
        for q in range(len(xlist)-1):
            plt.plot([int(xlist[q]),int(xlist[q+1])],[int(ylist[q]),int(ylist[q+1])])
        plt.axis([0,35,0,35])

        #if(r%20==0):
            #plt.show()

        #grid/=np.max(np.abs(grid),axis=0)
        #gridt=0
        #gridt=np.interp(grid, (grid.min(), grid.max()),(0,1))
        grid=grid/np.sum(grid)
        #rospy.loginfo(highpoint)
        rospy.loginfo(grid[highx][highy][highz])
        highz=highz+1
        r=r+1
        f.write("Message:") #writes to file
        f.write(str(r))
        f.write('\n')
        f.write("X:")
        f.write(str(highx))
        f.write('\n')
        f.write("Y:")
        f.write(str(highy))
        f.write('\n')
        f.write("Theta:")
        f.write(str(highz))
        f.write('\n')

        #grid=deepcopy(gridt)


        print('\n')
        #rospy.loginfo(grid[20][22][2])

        grid2=deepcopy(grid)
        #rospy.loginfo(grid2[20][22][2])







    #plt.show()
    return






if __name__ == '__main__':

    Read()


    #rospy.spin()
