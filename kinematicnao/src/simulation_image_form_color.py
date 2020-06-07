#! /usr/bin/env python
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
#import re
import matplotlib.pyplot as plt
#READ BELLOW TO ADAPT TO REAL NAO
#TO READAPT in case of change: width/height image(px), camera focal length(px), ball radius(mm), the H field of view (degree).

#the image Width and Height are fixed in the callback, if the scale change in the webots controller_python, the value in gray_callback have to be changed.
# The other parameters are in circle/ellipse fct (wanted to avoid global) .

object_present= False #To check if there is an object
position = Point()
my_ellipse_data=[]

def gray_callback(data):
    width=160 #x
    heigth=120 #y

    #the dtype is important to transform the data in an opencv image format
    gray_image = np.zeros([heigth,width], dtype=np.uint8)

    #transform the string into a 1D list
    gray_image_1D = data.data.split(' ')
    
    #transform the 1D list into a 2D array
    for x in range(0, width*heigth):
        gray_image[int(x/width)][x%width] = gray_image_1D[x]
    #take only the contour of the objects
    gray_image = cv2.Canny(gray_image,80,80)
    #cv2.imshow("ellipse",gray_image)
    #cv2.waitKey(1)
    my_ellipse_data=detect_ellipse(gray_image,width)
    #my_circle_data= detect_circles(gray_image,width)

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
    
    print(object_present)

    """
    cv2.imshow("image_gray",gray_image)
    cv2.waitKey(1)
    """

def detect_ellipse(imgThresholded,width):
    global object_present
    global my_ellipse_data
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
    my_ellipse_data=my_circle_data
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


def color_callback(data):
    global object_present
    global my_ellipse_data

    if object_present== True and my_ellipse_data!=[]:
        width=160 #x
        heigth=120 #y
        z=5

        my_ellipse= my_ellipse_data#to avoid that the other callback modify it while we work on it, i could use semaphore yes, but well
        position_pub = rospy.Publisher('/point', PointStamped, queue_size=1)#a Point stamped is expected, but here the id is useless. only the point will serve
        #the dtype is important to transform the data in an opencv image format
        color_image = np.zeros([heigth,width,3], dtype=np.uint8)
        
        #transform the string into a 1D list
        #color_image_1D = re.split(r'[[ ]]', data.data)
        color_image_1D = data.data.split(' ')

        #transform the 1D list into a 2D array BGR image
        for x in range(0, width*heigth*3):
            color_image[int(x/(width*3))][int(x/3)%(width)][x%3] = color_image_1D[x]
        
        #cv2.imshow("image_color",color_image)
        #cv2.waitKey(1)
        color=image_treatment(color_image,my_ellipse)
        if color != "red" and color!="none":
            position=PointStamped()
            position.point=Point(my_ellipse[0][3],my_ellipse[0][4], z)
            position.header.stamp = rospy.Time.now()
            position.header.frame_id = color
        
            position_pub.publish(position)
    
    
def image_treatment(color_image,my_ellipse_data):
    # define color range in RGB
    #used https://pinetools.com/image-color-picker
    
    lower_red = np.array([150,0,0])
    upper_red = np.array([255,160,140])

    lower_green = np.array([0, 100, 0])
    upper_green = np.array([119, 255, 130])

    lower_blue = np.array([60,60,120])
    upper_blue = np.array([120,120,255])

    lower_yellow = np.array([190,190,50])
    upper_yellow = np.array([255,255,220])
    
    #I take only a part of the ball as a pic
    img = color_image[ int(my_ellipse_data[0][1] - my_ellipse_data[0][2]/3) : int(my_ellipse_data[0][1] + my_ellipse_data[0][2]/4) , int(my_ellipse_data[0][0] - my_ellipse_data[0][2]/3) : int(my_ellipse_data[0][0] + my_ellipse_data[0][2]/4)]
    #cv2.imshow("litt_color",img)
    #cv2.waitKey(1)
    #I extract the mean color from the resized pic
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #Suppose to increase the brightness of the image by 50per have no idea how
    img= cv2.add(img,np.array([50.0]))
    #extract average color in RGB
    average = img.mean(axis=0).mean(axis=0)

    print(average)
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
    print(color)

    """
    for i in average: #Actually, this for is just not to see my comments
        
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
    #print(color)
    return color
    

def main():
    rospy.init_node('simulation_image', anonymous=True)
    position_pub = rospy.Publisher('/point', PointStamped, queue_size=1)#a Point stamped is expected, but here the id is useless. only the point will serve

    rospy.Subscriber('gray_image',String, gray_callback)
    rospy.Subscriber('blue_image',String, color_callback)
    if object_present==True:
        image_treatment()

    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
    rospy.spin()