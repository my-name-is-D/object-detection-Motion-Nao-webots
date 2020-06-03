#! /usr/bin/env python
import numpy as np
import pandas as pd
import math
import rospy
import cv2, PIL
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PointStamped, Point

#WARNING: the image Width and Height are fixed here, if the scale change in the webots controller_python, the value in gray_callback have to be changed.
prev_data=""
same=0

object_present= False #To check if there is an object
position = Point()

def gray_callback(data):
    global prev_data, same
    #print(data.data)
    width=160 #x
    heigth=120 #y
    #print(4799/60)
    gray_image = np.zeros([heigth,width], dtype=np.uint8)
    #print(len(gray_image))
    gray_image_1D = data.data.split(' ')
    
    #Testing unit
    """
    if data.data != prev_data:
        same=1+same
        prev_data=data.data
    print(same)
    """

    for x in range(0, width*heigth):
        gray_image[int(x/width)][x%width] = gray_image_1D[x]
    #print gray_image[heigth-1]

    cv2.imshow("image_gray",gray_image)
    cv2.waitKey(1)

    

def blue_callback(data):
    print(" ")


def image_treatment():
    print(" ")


def main():
    rospy.init_node('simulation_image', anonymous=True)
    position_pub = rospy.Publisher('/point', PointStamped, queue_size=1)#a Point stamped is expected, but here the id is useless. only the point will serve
    
    
    rospy.Subscriber('gray_image',String, gray_callback)
    #rospy.Publisher('blue_image',String, blue_callback)
    if object_present==True:
        image_treatment()

    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
    rospy.spin()