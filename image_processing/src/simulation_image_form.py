#! /usr/bin/env python
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point


#READ BELLOW TO ADAPT TO REAL NAO
#TO READAPT in case of change: width/height image(px), camera focal length(px), ball radius(mm), the H field of view (degree).

#the image Width and Height are fixed in the callback, if the scale change in the webots controller_python, the value in gray_callback have to be changed.
# The other parameters are in circle/ellipse fct (wanted to avoid global) .

object_present= False #To check if there is an object

def gray_callback(data):
    z= 12
    width=160 #x
    heigth=120 #y

    position_pub = rospy.Publisher('/point', PointStamped, queue_size=1)#a Point stamped is expected, but here the id is useless. only the point will serve

    #the dtype is important to transform the data in an opencv image format
    gray_image = np.zeros([heigth,width], dtype=np.uint8)

    #transform the string into a 1D list
    gray_image_1D = data.data.split(' ')
    
    #transform the 1D list into a 2D array
    for x in range(0, width*heigth):
        gray_image[int(x/width)][x%width] = gray_image_1D[x]
    #take only the contour of the objects
    gray_image = cv2.Canny(gray_image,80,80)
    
    my_ellipse_data=detect_ellipse(gray_image,width)
    #my_circle_data= detect_circles(gray_image,width)
    """
    try:

        #print ('ellipse: ',my_ellipse_data)
        #print ('circle: ',my_circle_data)
        #print 'centre x and y of the circle: ', my_circle_data[0][0],', ',my_circle_data[0][1]
        print 'centre x and y of the ellips: ', my_ellipse_data[0][0],', ',my_ellipse_data[0][1]
        #print 'radius circle: ',my_circle_data[0][2]
        print 'radius ellips: ',my_ellipse_data[0][2]
        #print 'distance ball circle (mm): ', my_circle_data[0][3]
        print 'distance ball ellips (mm): ', my_ellipse_data[0][3]
        print 'distY:',my_ellipse_data[0][4]
    except IndexError:
        pass
    """
    
    print(object_present)
    #Send data 
    if object_present==True and my_ellipse_data!=[] :
        #print("in")
        print my_ellipse_data[0][3]
        position=PointStamped()
        position.header.stamp = rospy.Time.now()
        position.header.frame_id = "ellipse"
        position.point=Point(my_ellipse_data[0][3],my_ellipse_data[0][4], z)
        position_pub.publish(position)
    
    """
    cv2.imshow("image_gray",gray_image)
    cv2.waitKey(1)
    """

def detect_ellipse(imgThresholded,width):
    global object_present

    ball_real_radius_mm= 69.7 #real: 69.7
    simulation_HFOV=45
    focallenght = 80#for the fictive frame 180x120
    my_circle_data=[]
    number_of_circles=0 #number of circles

    contours, hierarchy = cv2.findContours(imgThresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt) #to check size
        #print(area)
        if area < 40 or len(cnt) < 5 or area >7000:  #to get rid of the noises and interference 
            continue
        #(x,y),(MA,ma),angle = cv2.fitEllipse(cnt) #not practical to use
        ellipse = cv2.fitEllipse(cnt)

        rbox = cv2.fitEllipse(cnt) #we do a box arround the ellipse
        cv2.ellipse(imgThresholded, rbox, (100,0,255), 2,cv2.LINE_AA)

        Ma= ellipse[1][0] #long axe
        ma= ellipse[1][1] #little axe
        r1=Ma/2 
        r2=ma/2
        x=ellipse[0][0]
        y=ellipse[0][1]
        r=(((r1**2+r2**2)/2)**0.5) #we assume we have some kind of circle
        distance= focallenght* ball_real_radius_mm/r
        
        #angley = -1*((simulation_HFOV*(x/width)-(width/2))) #find the angle of view of the object (in x)
        #dY= distance*np.sin(angley*np.pi/180)
        dY=-1*(x- width/2)#no px rectification here because we use the simulation camera, each px are a perfect squared unit.
        number_of_circles=1+number_of_circles
        #print 'centre x and y of the circle: ', x,', ',y
        """
        print 'radius: ',r
        print 'distance ball (mm): ', distance
        """
        my_circle_data.append([x,y,r,distance,dY])
    
    #print 'number of ellipses detected',number_of_circles
    #print(my_circle_data)
    cv2.imshow("Ellipses", imgThresholded)
    cv2.waitKey(1)

    #Here we send a signal saying there is an object 
    if number_of_circles>0:
        object_present=True
    else:
        object_present=False

    return my_circle_data
 
def detect_circles(imgThresholded, width):
    global object_present
    ball_real_radius_mm= 69.7 #simulation ball
    simulation_HFOV=45
    focallenght = 75# for the fictive frame 180x120
    #imgThresholded=cv2.cvtColor(imgThresholded,cv2.COLOR_BGR2GRAY) #Convert the captured frame from a 3channel (BGR2HSV) to 1
    circles=cv2.HoughCircles(imgThresholded, cv2.HOUGH_GRADIENT,1, 40,param1=60, param2=20, minRadius=5, maxRadius=70) #Problem if coins are too close or too far
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
                    """
                    print 'centre x and y of the circle: ', x,', ',y
                    print 'radius: ',r
                    print 'distance ball (mm): ', distance
                    """
                    my_circle_data.append([x,y,r,distance,dY])


                except IndexError, ValueError: #in case i see nothing relevant (1st image actually)
                    pass
        except TypeError,ValueError: #in case there is no circles
            pass
    #print 'number of circles detected',number_of_circles
    #Here we send a signal saying there is an object 
    if number_of_circles>0:
        object_present=True
    else:
        object_present=False
    #print(my_circle_data)
    cv2.imshow("CIRCLES", imgThresholded)
    cv2.waitKey(1)



    return my_circle_data

def blue_callback(data):
    print(" ")


def image_treatment():
    print(" ")


def main():
    rospy.init_node('simulation_image', anonymous=True)
    position_pub = rospy.Publisher('/point', PointStamped, queue_size=1)#a Point stamped is expected, but here the id is useless. only the point will serve

    rospy.Subscriber('gray_image',String, gray_callback)
    #rospy.Subscriber('blue_image',String, blue_callback)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    rospy.spin()