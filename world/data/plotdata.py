#! /usr/bin/env python
from __future__ import division
import csv
import os

import matplotlib.pyplot as plt
import numpy as np
import math


def armcsv():
    time,LshoulderP,LshoulderR = [], [], []
    LelbowY, LelbowR,LwristY =[],[],[]
    
    with open('/home/cata/nao_ws/src/world/data/arm.csv') as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=',')
        for row in csv_reader:
            time.append(float(row['timestamp']))
            LshoulderP.append(float(row['LshoulderP']))
            LshoulderR.append(float(row['LshoulderR']))
            LelbowY.append(float(row['LelbowY']))
            LelbowR.append(float(row['LelbowR']))
            LwristY.append(float(row['LwristY']))
    return time,LshoulderP,LshoulderR,LelbowY, LelbowR,LwristY

def handcsv():
    timehand=[]
    Lphalanx1,Lphalanx5,Lphalanx2,Lphalanx3,Lphalanx4,Lphalanx6,Lphalanx7,Lphalanx8= [],[],[],[],[],[],[],[]
    
    with open('/home/cata/nao_ws/src/world/data/hand.csv') as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=',')
        for row in csv_reader:
            timehand.append(float(row['timestamp']))
            Lphalanx1.append(float(row['LPhalanx1']))
            Lphalanx2.append(float(row['LPhalanx2']))
            Lphalanx3.append(float(row['LPhalanx3']))
            Lphalanx4.append(float(row['LPhalanx4']))
            Lphalanx5.append(float(row['LPhalanx5']))
            Lphalanx6.append(float(row['LPhalanx6']))
            Lphalanx7.append(float(row['LPhalanx7']))
            Lphalanx8.append(float(row['LPhalanx8']))
            
    return timehand,Lphalanx1,Lphalanx5,Lphalanx2,Lphalanx3,Lphalanx4,Lphalanx6,Lphalanx7,Lphalanx8

def main():
    plt.style.use('classic')
    #ax = plt.subplots()

    time,LshoulderP,LshoulderR,LelbowY, LelbowR,LwristY=armcsv()
    timehand,Lphalanx1,Lphalanx5,Lphalanx2,Lphalanx3,Lphalanx4,Lphalanx6,Lphalanx7,Lphalanx8=handcsv()
    """
    plt.plot(timehand,Lphalanx1,'-b', label='Lphalanx1/rad')
    plt.plot(timehand,Lphalanx2,'-r', label='Lphalanx2/rad')
    plt.plot(timehand,Lphalanx3,'-r', label='Lphalanx3/rad')
    plt.legend(framealpha=1, frameon=False)
    plt.show()
    """
    plt.scatter(time, LshoulderP, color='blue')
    plt.plot(time,LshoulderP,'-b', label='LShoulderPitch/rad')
    plt.scatter(time, LshoulderR, color='red')
    plt.plot(time,LshoulderR,'--r', label='LShoulderRoll/rad')
    plt.legend(framealpha=1, frameon=False)
    plt.title('Shoulder motion')
    plt.ylabel('motion (rad)')
    plt.xlabel('time (s)')
    plt.show()
    plt.scatter(time, LelbowY, color='green')
    plt.plot(time,LelbowY,'--g', label='LelbowYaw/rad')  
    plt.scatter(time, LelbowR, color='red') 
    plt.plot(time,LelbowR,'-r', label='LelbowRoll/rad')
    plt.legend(framealpha=1, frameon=False)   
    plt.title('Elbow motion')
    plt.ylabel('motion (rad)')
    plt.xlabel('time (s)')
    plt.show() 
    
    


if __name__ == '__main__':
    main()
