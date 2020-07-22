#! /usr/bin/env python
from __future__ import division #to have 1/10=0.1 and not 0

import numpy as np
import rospy
import math
import cv2
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PointStamped, Point
import matplotlib.pyplot as plt

#READ BELLOW TO READAPT CODE (TO REAL NAO for eg)
#TO READAPT in case of change: width/height image(px), camera focal length(px), ball radius(mm), the H field of view (degree).

#the image Width and Height are fixed in the callback, if the scale change in the webots controller_python, the value in gray_callback have to be changed.
# The other parameters are in circle/ellipse fct (wanted to avoid global) .

object_present= False #To check if there is an object
position = Point()
my_ellipse_data=[]

def gray_callback(data):
    width=160 #x
    heigth=120 #y
    kernel1 = np.ones((1,1),np.uint8)
    kernel4 = np.ones((4,4),np.uint8)
    kernel3 = np.ones((3,3),np.uint8)
    #the dtype is important to transform the data in an opencv image format
    gray_image = np.zeros([heigth,width], dtype=np.uint8)

    #transform the string into a 1D list
    gray_image_1D = data.data.split(' ')
    
    #transform the 1D list into a 2D array
    for x in range(0, width*heigth):
        gray_image[int(x/width)][x%width] = gray_image_1D[x]
    
    saved=gray_image
    #try to erase noise

    #cv2.imshow("first", gray_image)
    #cv2.waitKey(1)
    gray_image = cv2.GaussianBlur(gray_image,(1,1),0)
    #"""
    th2 = cv2.adaptiveThreshold(gray_image,255, cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY_INV,25,5)
    
    #cv2.imshow("check threshold", th2)

    #cv2.waitKey(1)
    th2= cv2.erode(th2,kernel4, iterations = 3)
    th2= cv2.erode(th2,kernel3, iterations = 1)
    th2= cv2.dilate(th2,kernel4, iterations = 5)
    #th2= cv2.dilate(th2,kernel3, iterations = 1)
    th2= cv2.dilate(th2,kernel1, iterations = 20)
    #cv2.imshow("check mask", th2)

    #cv2.waitKey(1)
    contours, hierarchy = cv2.findContours(th2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    th2=cv2.drawContours(th2, contours, -1, (255,255,255), 3)
    #cv2.inpaint
    #mark = np.zeros(th2.shape[:2], np.uint8)
    res = cv2.inpaint(gray_image, th2,50, cv2.INPAINT_TELEA)
    gray_image = cv2.GaussianBlur(res,(3,3),0)
    #"""

    #take only the contour of the objects
    gray_image = cv2.Canny(gray_image,80,80)
    gray_image = cv2.erode(gray_image,kernel1, iterations = 10)
    gray_image = cv2.dilate(gray_image,kernel4,iterations = 4)
    gray_image = cv2.erode(gray_image,kernel4, iterations = 4)
    #cv2.imshow("th2", gray_image)
    #Specify size on vertical axis
    vertical = gray_image.copy()
    verticalsize = vertical / 30
    verticalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    #Apply morphology operations
    cv2.erode(vertical, vertical, verticalStructure, iterations = 1)
    cv2.dilate(vertical, vertical, verticalStructure, iterations = 1)
    #Show extracted vertical lines
    
    #cv2.imshow("vertical", vertical)
    #cv2.imshow("image_gray",gray_image)
    #cv2.waitKey(1)



    my_ellipse_data=detect_ellipse(gray_image,width,saved)
    #my_ellipse_data= detect_circles(gray_image,width)

    


    

def detect_ellipse(imgThresholded,width,saved):
    global object_present
    global my_ellipse_data

    ball_real_radius_mm= 10.0 #in mm
    simulation_HFOV=45
    focallenght = 182#for the fictive frame 180x120
    
    
    my_circle_data=[]
    number_of_circles=0 #number of circles

    #extract the contour out of the grayscale
    contours, hierarchy = cv2.findContours(imgThresholded, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    """
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, .03 * cv2.arcLength(cnt, True), True)
        print len(approx)
        if len(approx)==3:
            print "triangle"
            #cv2.drawContours(saved,[cnt],0,(122,212,78),-1)
        if len(approx)==4:
            print "sqaure"
            #cv2.drawContours(saved,[cnt],0,(0,112,122),-1)
        if len(approx)==6:
            print "shape"
            cv2.drawContours(saved,[cnt],0,(0,0,122),-1)
        if len(approx)==8:
            print "circle"
            cv2.drawContours(saved,[cnt],0,(200,0,122),-1)
    
    cv2.imshow("image shapes",saved)
    cv2.waitKey(1)
    """
    #print(hierarchy)
    
    imgThresholded=cv2.drawContours(imgThresholded, contours, -1, (255,255,255), 3)
    tempo_info=[]
    n=0
    for cnt in contours:
        #print ("cnt", cnt)
        area = cv2.contourArea(cnt) #to check size
        #print(area)
        if area < 10 or len(cnt) < 10 or area > 7000:  #to get rid of the noises and false positive
            continue
        
        n+=1
        if n==1: #don't want several ellipses
        #cv2.drawContours(saved,[cnt],0,(0,0,122),-1)

    #//THE ONE interesting IS N==1 (the contour of the circle above)

            
            """

            rect=cv2.minAreaRect(cnt)

            cx, cy = rect[0]
            w, h = rect[1]
            theta = rect[2]

            print("cx,xy,w,h,theta", cx, cy, w, h, theta)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(saved, [box], 0, 255, -1)
            cv2.imshow("image shapes",saved)
            cv2.waitKey(1)
            """
            #(x,y),(MA,ma),angle = cv2.fitEllipse(cnt) #not practical to use
            ellipse = cv2.fitEllipse(cnt)

            rbox = cv2.fitEllipse(cnt) #we do a box arround the ellipse
            
            cv2.ellipse(imgThresholded, rbox, (100,0,255), 2,cv2.LINE_AA)

            Ma= ellipse[1][0] #long axe
            ma= ellipse[1][1] #little axe
        
            x=ellipse[0][0]
            y=ellipse[0][1]
                    
            r1=Ma/2 #radius 1
            r2=ma/2 #radius 2 (it's an ellipse, not a circle)
            r=(((r1**2+r2**2)/2)**0.5) #we assume we have some kind of circle
            
            distance = 5.49501*Ma - 7.77369*ma + 250.85954#focallenght* ball_real_radius_mm/r
            
            #angley = -1*((simulation_HFOV*(x/width)-(width/2))) #find the angle of view of the object (in x)
            #dY= distance*np.sin(angley*np.pi/180)
            dY=-1*(x- width/2)#no px rectification here because we use the simulation camera, each px are a perfect squared unit.
            number_of_circles=1+number_of_circles    
            my_circle_data.append([x,y,r,distance,dY])
            my_ellipse_data=my_circle_data
            tempo_info.append([Ma,ma,x,y])
            print ("distance MA,ma,x,y    :  ",distance,Ma,ma,x,y)
      
    

    cv2.imshow("Ellipses", imgThresholded)
    cv2.waitKey(1)

    #Here we send a signal saying there is an object 
    if number_of_circles>0:
        object_present=True
    else:
        object_present=False

    return my_circle_data
"""
function not used, alternative to ellipse
""" 
def detect_circles(imgThresholded, width):
    global object_present
    global my_ellipse_data
    ball_real_radius_mm= 10 #simulation ball
    simulation_HFOV=45
    focallenght = 182# for the fictive frame 180x120
    #imgThresholded=cv2.cvtColor(imgThresholded,cv2.COLOR_BGR2GRAY) #Convert the captured frame from a 3channel (BGR2HSV) to 1
    
    circles=cv2.HoughCircles(imgThresholded, cv2.HOUGH_GRADIENT,1, 20,param1=100, param2=5, minRadius=2, maxRadius=30) #Problem if coins are too close or too far
    number_of_circles=0 #number of circles
    my_circle_data=[]

    if circles is not None:

        #circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        try:
            #for (x, y, r) in circles: 
            for i in range (0,len(circles)):
                x=circles[0][i][0]
                y=circles[0][i][1]
                r=circles[0][i][2]
                cv2.circle(imgThresholded, (x, y), r, (100, 0, 255), 4)

                try:

                    distance= focallenght* ball_real_radius_mm/r
                    #print("size_circle")   
                    #print(size_circle) 
                    number_of_circles=1+number_of_circles  
                    #angley = -1*((simulation_HFOV*(x/width)-(width/2))) #find the angle of view of the object (in x)
                    #dY= distance*np.sin(angley*np.pi/180)
                    dY=-1*(x- width/2)#no px rectification here because we use the simulation camera, each px are a perfect squared unit.
                    
                    print 'centre x and y of the circle: ', x,', ',y
                    print 'radius: ',r
                    print 'distance ball (mm): ', distance
                
                    my_circle_data.append([x,y,r,distance,dY])


                except IndexError, ValueError: #in case i see nothing relevant (1st image actually)
                    pass
        except TypeError,ValueError: #in case there is no circles
            pass
    #print 'number of circles detected',number_of_circles
    #Here we send a signal saying there is an object 
    my_ellipse_data=my_circle_data
    if number_of_circles>0:
        object_present=True
    else:
        object_present=False
    #print(my_circle_data)
    cv2.imshow("CIRCLES", imgThresholded)
    cv2.waitKey(1)




    return my_circle_data


def color_callback(data):
    global object_present
    global my_ellipse_data

    if object_present== True and my_ellipse_data!=[]:
        width=160 #x
        heigth=120 #y
        z=0

        my_ellipse= my_ellipse_data#to avoid that the other callback modify it while we work on it, i could use semaphore yes, but well
        
        position_pub = rospy.Publisher('/point', PointStamped, queue_size=1)#a Point stamped is expected, but here the id is useless. only the point will serve
        #the dtype is important to transform the data in an opencv image format
        color_image = np.zeros([heigth,width,3], dtype=np.uint8)
        
        color_image_1D = data.data.split(' ')

        #transform the 1D list into a 2D array BGR image
        for x in range(0, width*heigth*3):
            color_image[int(x/(width*3))][int(x/3)%(width)][x%3] = color_image_1D[x]
        
        #extract the ball color
        color=image_treatment(color_image,my_ellipse)

        #check if we are in the workspace and color is stimulating
        workspace_check(color,my_ellipse,z)

        #if color != "red" and color!="none":
        position=PointStamped()
        position.point=Point(my_ellipse[0][3],my_ellipse[0][4],z)
        position.header.stamp = rospy.Time.now()
        position.header.frame_id = color
    
        position_pub.publish(position)
       
        
        
    
def image_treatment(color_image,my_ellipse_data):
    # define color range in RGB
    #used https://pinetools.com/image-color-picker
    
    lower_red = np.array([150,0,0])
    upper_red = np.array([255,160,140])

    lower_yellow = np.array([190,190,50])
    upper_yellow = np.array([255,255,220])

    lower_green = np.array([0, 100, 0])
    upper_green = np.array([119, 255, 119])

    lower_blue = np.array([60,60,120])
    upper_blue = np.array([120,120,255])
 
    #I take only a part of the ball as a pic
    img = color_image[ int(my_ellipse_data[0][1] - my_ellipse_data[0][2]/3) : int(my_ellipse_data[0][1] + my_ellipse_data[0][2]/4) , int(my_ellipse_data[0][0] - my_ellipse_data[0][2]/3) : int(my_ellipse_data[0][0] + my_ellipse_data[0][2]/4)]
    
    #I extract the mean color from the resized pic
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #Iincrease the brightness of the image by 50per 
    img= cv2.add(img,np.array([50.0]))
    #extract average color in RGB
    average = img.mean(axis=0).mean(axis=0)

    check=0
    #we check if the average is in the boundaries
    for lower,aver,upper in zip(lower_red,average,upper_red):
        if lower > aver or aver > upper:
            check+=1
    if check ==0:
        color="red"
    else:
        check=0
        for lower,aver,upper in zip(lower_yellow,average,upper_yellow):
            if lower > aver or aver > upper:
                check+=1
        if check ==0:
            color="yellow"    
    if check>0:
        check=0
        for lower,aver,upper in zip(lower_blue,average,upper_blue):
            if lower > aver or aver > upper:
                check+=1
        if check ==0:
            color="blue"
    if check>0:
        check=0
        for lower,aver,upper in zip(lower_green,average,upper_green):
            if lower > aver or aver > upper:
                check+=1
        if check ==0:
            color="green"

        else:
            color="none"
    

    """
    #TESTS 
    for i in blabla: #Actually, this for is just not to see my comments
        
        #TEST 1 to get the dominant color (and average color)
        pixels = np.float32(img.reshape(-1, 3))

        n_colors = 5
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, .1)
        flags = cv2.KMEANS_RANDOM_CENTERS

        _, labels, palette = cv2.kmeans(pixels, n_colors, None, criteria, 10, flags)
        _, counts = np.unique(labels, return_counts=True)
        dominant = palette[np.argmax(counts)]
        indices = np.argsort(counts)[::-1]   
        freqs = np.cumsum(np.hstack([[0], counts[indices]/counts.sum()]))
        rows = np.int_(img.shape[0]*freqs)

        avg_patch = np.ones(shape=img.shape, dtype=np.uint8)*np.uint8(average)
        
        indices = np.argsort(counts)[::-1]   
        freqs = np.cumsum(np.hstack([[0], counts[indices]/counts.sum()]))
        rows = np.int_(img.shape[0]*freqs)

        dom_patch = np.zeros(shape=img.shape, dtype=np.uint8)
        for i in range(len(rows) - 1):
            dom_patch[rows[i]:rows[i + 1], :, :] += np.uint8(palette[indices[i]])

        fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(12,6))
        ax0.imshow(avg_patch)
        ax0.set_title('Average color')
        ax0.axis('off')
        ax1.imshow(dom_patch)
        ax1.set_title('Dominant colors')
        ax1.axis('off')
        plt.show(fig)
        
        boundaries = [
        ([17, 15, 100], [50, 56, 200]),
        ([86, 31, 4], [220, 88, 50]),
        ([25, 146, 190], [62, 174, 250]),
        ([103, 86, 65], [145, 133, 128])
        ]
        #cv2.imshow("litt_color",average)
        #cv2.waitKey(1)
        
        #TEST 2 to get the dominant color
        
        a2D = img.reshape(-1,img.shape[-1])
        col_range = (256, 256, 256) # generically : a2D.max(0)+1
        a1D = np.ravel_multi_index(a2D.T, col_range)
        result=np.unravel_index(np.bincount(a1D).argmax(), col_range)
        
        # Threshold the HSV image to get only copper colors
        mask_red = cv2.inRange(img, lower_red, upper_red)
        mask_green = cv2.inRange(img, lower_green, upper_green)
        k = cv2.waitKey(5) & 0xFF
        
    """
   
    return color
    
def workspace_check(color,my_ellipse,z):
    pub_stimulation = rospy.Publisher('/stimulation', Float64, queue_size=5)

    stimulation=0
    execution= True
    #kinematic sizes, would be cleaner to get them from KinematicsDefines.h file (but that was faster, will do it later)
    ShoulderOffsetZ=75.0
    ShoulderOffsetY=98.0
    ElbowOffsetY=15.0
    UpperArmLength=105.0
    LowerArmLength=57.7
    HandOffsetX=55.95
    HandOffsetZ=12.31
    Dist_arm=HandOffsetX+UpperArmLength+LowerArmLength #219
    OffsetLY= ShoulderOffsetY+ElbowOffsetY#113
    OffsetLZ=ShoulderOffsetZ+HandOffsetZ #112

    lpx=my_ellipse[0][3]
    lpy=my_ellipse[0][4]
    lpz=z
    """
    print(color)
    print(lpx,lpy,lpz)
    """

    #np.warnings.filterwarnings('ignore')#to ignore the "invalid value if arcsin not possible"
    
    #To consider the Offset of y (on y, the dist max is 331.7 while on x it's 218)
    #on z the dist max is 112+ 218 (from the origin, not from shoulder)
    second_Dist_arm= Dist_arm + OffsetLY
    third_Dist_arm=Dist_arm +OffsetLZ

    #IF COLOR NOT STIMULATING OR Y AND X TOO LOW OR HIGH --> no stimulation
    if (color == "red" or color == "none" or color=="yellow" or lpy<0 or lpx>Dist_arm) :
        execution= False
        #print("clor",color)
        #print("lpx and dist",lpx, Dist_arm)
    # Nao Workspace check
    try:
        if not (not np.isnan(math.acos(lpx/Dist_arm)) and not np.isnan(math.asin((lpy-OffsetLY)/Dist_arm)) and not np.isnan(math.acos((lpz-OffsetLZ)/Dist_arm))
        and not np.isnan(math.asin(lpx/Dist_arm)) ) :
            execution= False
            #print("1")
        elif not(math.sqrt(lpx*lpx+(lpy-OffsetLY)*(lpy-OffsetLY))<= second_Dist_arm and 0<= math.acos(lpx/Dist_arm) and math.acos(lpx/Dist_arm)<=math.acos(0/Dist_arm)):
            execution= False
            #print("dist arm", second_Dist_arm)
            #print("dist x and y",math.sqrt(lpx*lpx+(lpy-OffsetLY)*(lpy-OffsetLY)))
        elif not (math.asin((-67-OffsetLY)/Dist_arm)<= math.asin((lpy-OffsetLY)/Dist_arm) and math.asin((lpy-OffsetLY)/Dist_arm)<=math.asin((331-OffsetLY)/Dist_arm)):
            #0 correspond to max point when x=Dist_arm)
            #-67 is the min y pose and 331 is the max (with offset)
            execution= False
            #print("2")
        elif not(math.sqrt(lpx*lpx+(lpz-OffsetLZ)*(lpz-OffsetLZ))<= third_Dist_arm):
            execution= False
            #print("3")
        elif not( math.acos((305.9-OffsetLZ)/Dist_arm)<= math.acos((lpz-OffsetLZ)/Dist_arm) and
        math.acos((lpz-OffsetLZ)/Dist_arm)<=math.acos((-80-OffsetLZ)/Dist_arm) and math.asin(0/Dist_arm)<=math.asin(lpx/Dist_arm)):
            #We don't let x go behind its back, 306 and -80 are the max and min z with offset.
            execution= False
            #print("4")

    except ValueError: #if values are too high
        execution= False
        #print("here")

    if execution == False:
        print("Object not interesting : 0.0012A")
        stimulation=0.0012
    else:
        print("Object interesting : 0.0035A")
        stimulation=0.0035
    pub_stimulation.publish(stimulation)
    

def main():
    rospy.init_node('simulation_image', anonymous=True)
    position_pub = rospy.Publisher('/point', PointStamped, queue_size=1)#a Point stamped is expected, but here the id is useless. only the point will serve

    rospy.Subscriber('gray_image',String, gray_callback)
    rospy.Subscriber('blue_image',String, color_callback)

    cv2.destroyAllWindows()
    rospy.sleep(1)
if __name__ == '__main__':
    main()
    rospy.spin()
