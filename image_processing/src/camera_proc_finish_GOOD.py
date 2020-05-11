#!/usr/bin/env python
from __future__ import print_function

"""LIST OF FUNCTION (IN ORDER):

color_mask() (not used): keep everything blue in the picture
aruco_detection(): the aruco detection
calcul_distance() : to get the distance once the marker detected
detect_circles() : same logic used in both
detect_ellipse()

check_copper, check_silver, check gold(): 3 filter to keep only one range of color.
image_modification_silver, image_modification_gold, image_modifications : where i put all the functions that transform the image

Callback: where all the principal functions are called
Handler(): the function launched after a "CTRL-C"

main

"""



#too many libaries now
import signal
import time
import roslib
roslib.load_manifest('image_processing')
import sys
import rospy
import cv2, PIL
from cv2 import aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib as mpl 
import matplotlib.pyplot as plt
import pandas as pd

my_coins=[]
no_go_zone=[]#has to have a memory because of the loops. Problem if the camera is really moving and 1 piece get in a point already registered
index=0 #to have an index in my matrice
my_matrix_check=[]#to get the value most seen at 1 position, keep track of value of coins seen in the proximity of another one previously seen
height=0 #size of my matric_check
focallenght=982.5# USb_camera #627 #computer camera   calculed with a picture of the marker and a coin and a known distance
old_distance =1 #used in case no aruco marker detected for a while
count=0 #used to refresh my_coins if camera moves, coins won't be at same place


#coins library : min size authorised for recognition, max size, according value
coins=np.array([[28,29,200],
          [22,23.2,100],
          [26.8,27.9,50],
          [20.8,21.8,20],
          [23.6,25.1,10],
          [17.3,18.5,5],
          [25.2,26.6,2],
          [19.5,21.5,1]]) #harder to detect for some reason


#NOT USED, it was to erase everything BUT the background. First test
def color_mask(frame):
    #what it keeps (blue filter)
    boundaries = [([120, 20,10],[255, 255, 255])]

    for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
      lower = np.array(lower, dtype = "uint8")
      upper = np.array(upper, dtype = "uint8")
    # check colors in boundaries 
    cv_image = cv2.inRange(frame, lower, upper)

    cv_image = cv2.GaussianBlur(cv_image,(15,15),0)
    kernel4 = np.ones((4,4),np.uint8)
    cv_image = cv2.erode(cv_image,kernel4, iterations = 1)
    cv_image = cv2.dilate(cv_image,kernel4,iterations = 1)
    cv_image = cv2.Canny(cv_image,100,200)
    return cv_image

def aruco_detection(cv_image):
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids

def calcul_distance(frame,corners,Size_marker):
    global no_go_zone
    corner = np.array(corners) #to get a real array
    corner=corner.reshape(4,2) #to get rid of the triple []

    no_go_zone=corner

    size_marker_x_px= abs(corner[0][0]-corner[1][0])
    size_marker_y_px= abs(corner[0][1]-corner[1][1])
    
    #This is only valid if the marker is positionned straight and perpendicular to the camera (4 configurations ok)
    if size_marker_x_px> size_marker_y_px:
          Size_marker_px = size_marker_x_px
    else:
          Size_marker_px=size_marker_y_px

    distance= focallenght*Size_marker/Size_marker_px

    return distance
   
def detect_circles(imgThresholded,cv_image,distance,color_check):
  global index
  global height
  #imgThresholded=cv2.cvtColor(imgThresholded,cv2.COLOR_BGR2GRAY) #Convert the captured frame from a 3channel (BGR2HSV) to 1
  circles=cv2.HoughCircles(imgThresholded, cv2.HOUGH_GRADIENT,1, 90,param1=60, param2=20, minRadius=13, maxRadius=40) #Problem if coins are too close or too far

  if circles is not None: #if no circle then don't do it
      circles = np.round(circles[0, :]).astype("int")

      if color_check=="copper":
        coin_check=range(6,8)
      elif color_check=="silver":
        coin_check=range(0,6)
      elif color_check=="gold":
        coin_check=range(0,3)

  number_of_circles=0 #number of circles
  coin_already_seen=0
  # loop over the (x, y) coordinates and radius of the circles
  try:
    for (x, y, r) in circles: 
      cv2.circle(imgThresholded, (x, y), r, (255, 255, 255), 4)
      
      try:
        size_circle= r*2*distance/focallenght
        #print("size_circle")   
        #print(size_circle)   
        print("x and y")
        print(x, y)
        print("r")
        print(r)
        number_of_circles+=1

        #check similarity with my library of coins

        
        for row in coin_check : #go through my library of coins wich has 8 coins registered in
          
          if size_circle >= coins[row][0] and size_circle <= coins[row][1]: #check if the size is in a range acceptable to be recognized as this coin
            for i in range(0,index): #check if there isn't been another circle detected at this point
                if color_check== my_coins[i][6]:
                    if my_coins!=[] and (x ==my_coins[i][1] or y==my_coins[i][2]): #exactly same x= loop = video reboot. probability of that happening with real feed: small 
                        coin_already_seen=1 
                    for h in range(0,height): #to check, not to put multiple time the same coin in the matrix_check (loop fault)
                        if my_matrix_check!=[] and (abs(x==my_matrix_check[h][1]) and abs(y==my_matrix_check[h][2])):
                            coin_already_seen=1
        
                    if my_coins!=[] and coin_already_seen!=1 and (abs(x-my_coins[i][1])<9 and abs(y-my_coins[i][2]<9)) : # It's a problem if 2 coins get at another one previous location in a real feed
                        coin_already_seen=1           
                        two_coins_1place=[i,x,y,coins[row][2]] 
                        my_matrix_check.append(two_coins_1place)# keep this in memory to check.
                        height+=1 #keep this matrix height in memory
                        #even like that risk of not getting the right coin with the average 
            #if x hasn't been already covered
            if coin_already_seen!=1 :
              this_coin=[index,x,y,coins[row][2],size_circle, r,color_check,"_"] 
              my_coins.append(this_coin) #then keep it in memory
              index+=1  #keep the size of this array in memory
               
      except IndexError: #in case i see nothing relevant (1st image actually)
        pass
  except TypeError: #in case there is no circles
    pass
  print("number of circles detected")
  print(number_of_circles)
  for row in range(0,index):
        cv2.putText(cv_image,"{}".format(my_coins[row][3]),(int(my_coins[row][1]),int(my_coins[row][2])),cv2.FONT_HERSHEY_PLAIN,1.5,(0,0,0),thickness=2,lineType=cv2.LINE_AA)     
  cv2.imshow("Image window2", cv_image)
  return circles


def detect_ellipse(imgThresholded,cv_image,distance,color_check):
  global index
  global height
  _,contours, hierarchy = cv2.findContours(imgThresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  if color_check=="copper":
    coin_check=range(6,8)
  elif color_check=="silver":
    coin_check=range(0,6)
  elif color_check=="gold":
    coin_check=range(0,6)

  number_of_circles=0 #number of circles

  for cnt in contours:
    area = cv2.contourArea(cnt) #to check if size MAYBE a coin to begin with (zoom taken into account)

    if area < 900 or len(cnt) < 5 or area >5000:  #to get rid of the noises and interference 
      continue
    #(x,y),(MA,ma),angle = cv2.fitEllipse(cnt) #not practical to use
    ellipse = cv2.fitEllipse(cnt)
  
    rbox = cv2.fitEllipse(cnt)
    cv2.ellipse(cv_image, rbox, (0,0,0), 2,cv2.LINE_AA)

    Ma= ellipse[1][0] #long axe
    ma= ellipse[1][1] #little axe
    r1=Ma/2 
    r2=ma/2
    x=ellipse[0][0]
    y=ellipse[0][1]
    r=(((r1**2+r2**2)/2)**0.5) #we assume we have some kind of circle

    size_circle= r*2*distance/focallenght
    print("x and y")
    print(x, y)
    print("r")
    print(r)
    print("size")
    print(size_circle)
    number_of_circles+=1

#we see the marker too well with silver so let's not count it like that
    if color_check=="silver":
      if no_go_zone!=[] and (x>no_go_zone[0][0] and x<no_go_zone[1][0]) and (y>no_go_zone[0][1] and y<no_go_zone[1][1]): #if circle detected on the marker
        size_circle=0 #to avoid unecessary variables     

    for row in coin_check : #go through my library of coins wich has 8 coins registered in
        coin_already_seen=0    
        if size_circle >= coins[row][0] and size_circle <= coins[row][1]: #check if the size is in a range acceptable to be recognized as this coin
          
          for i in range(0,index): #check if there isn't been another circle detected at this point
            if color_check== my_coins[i][6]:
                  
              if (x ==my_coins[i][1] or y==my_coins[i][2]): #exactly same x= loop = video reboot. probability of that happening with real feed: small 
                  coin_already_seen=1 
              #if (abs(x-my_coins[i][1])<9 and abs(y-my_coins[i][2]<9)):
                #   coin_already_seen=1

              for h in range(0,height): #to check, not to put multiple time the same coin in the matrix_check (loop fault)
                  if my_matrix_check!=[] and (x==my_matrix_check[h][1]) and (y==my_matrix_check[h][2]):
                      coin_already_seen=1
              
              if coin_already_seen!=1 and (abs(x-my_coins[i][1])<9 and abs(y-my_coins[i][2]<9)) : # It's a problem if 2 coins get at another one previous location in a real feed
                  coin_already_seen=1           
                  two_coins_1place=[i,x,y,coins[row][2]] 
                  my_matrix_check.append(two_coins_1place)# keep this in memory to check.
                  height+=1 #keep this matrix height in memory
                  #even like that risk of not getting the right coin with the average 
                  
          #if x hasn't been already covered
          if coin_already_seen!=1 :
            this_coin=[index,x,y,coins[row][2],size_circle, r,color_check,"_"] 
            my_coins.append(this_coin) #then keep it in memory
            index+=1  #keep the size of this array in memory
      
  print("number of ellipses detected")
  print(number_of_circles)
  #cv_txt=cv_image.copy()

  #cv2.imshow("Image window3", cv_txt)
  for row in range(0,index):
    cv2.putText(cv_image,"{}".format(my_coins[row][3]),(int(my_coins[row][1]),int(my_coins[row][2])),cv2.FONT_HERSHEY_PLAIN,1.5,(0,0,0),thickness=2,lineType=cv2.LINE_AA)     
  cv2.imshow("Image window3", cv_image)
  return cv_image # cv_txt



def check_copper(cv_image,distance):
  cv_image = cv2.cvtColor(cv_image.copy(), cv2.COLOR_BGR2HSV) #HSV has 3channels not good with circle.hough
  boundaries = [([0, 100, 30], [15, 255, 255])] #boundary of the copper's color

  for (lower, upper) in boundaries:
    # create the numpy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    # To get the colors in the boundaries 
  
    imgThreshold = cv2.inRange(cv_image, lower, upper)
    imgThreshold= (imgThreshold)

    color_check="copper"
    #detect_circles(imgThreshold,cv_image,distance,color_check)
    cv_image=detect_ellipse(imgThreshold,cv_image,distance,color_check)
    return cv_image

def check_silver(cv_image, distance):
  cv_image = cv2.cvtColor(cv_image.copy(), cv2.COLOR_BGR2HSV)
  boundaries = [([0, 0, 0], [140, 60, 240])] #if i change the max i can detect copper but get some things more accurate
  # loop over the boundaries
  for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    imgThreshold = cv2.inRange(cv_image, lower, upper)
    cv2.imshow("this_is_silver?",imgThreshold)
    imgThreshold= image_modifications_silver(imgThreshold)

    color_check="silver"
   
    #detect_circles(imgThreshold,cv_image,distance,color_check)
    cv_image=detect_ellipse(imgThreshold,cv_image,distance,color_check)
    return cv_image
  

def check_gold(cv_image, distance):
  cv_image = cv2.cvtColor(cv_image.copy(), cv2.COLOR_BGR2HSV)
  boundaries = [([20, 50, 10], [90,230,230])]#[115, 155, 155])]
  # loop over the boundaries
  for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    imgThreshold = cv2.inRange(cv_image, lower, upper)
    cv2.imshow("this_is_gold?",imgThreshold)
    imgThreshold= image_modifications_gold(imgThreshold)

    color_check="gold"
   
    #detect_circles(imgThreshold,cv_image,distance,color_check)
    cv_image=detect_ellipse(imgThreshold,cv_image,distance,color_check)
    return cv_image

def image_modifications_gold(imgThresholded):
    #imgThresholded=cv2.blur(imgThresholded, (3,3))

    # Taking a matrix of size 5 or 3 or 2as the kernel 
    kernel = np.ones((5,5), np.uint8)
    kernel2 = np.ones((3,3), np.uint8) 
    kernel3=np.ones((2,2), np.uint8) 

    imgThresholded=cv2.dilate(imgThresholded, kernel2, iterations=2) #high number to get rid of the little details on the coins
    imgThresholded=cv2.erode(imgThresholded, kernel2, iterations=2) # erode the white parts

    #imgThresholded=cv2.dilate(imgThresholded, kernel2, iterations=2) #dilate the white parts NOTE: if iteration 1 or 2 --> big difference over the circle result
    imgThresholded=cv2.erode(imgThresholded, kernel2, iterations=1)
    #imgThresholded=cv2.erode(imgThresholded, kernel3, iterations=2)

    #imgThresholded = cv2.Canny(imgThresholded,80,80)

    imgThresholded=cv2.dilate(imgThresholded, kernel2, iterations=1) #dilate the white parts
    imgThresholded=cv2.erode(imgThresholded, kernel2, iterations=1)
    cv2.imshow("this is my black_gold",imgThresholded)

    imgThresholded = cv2.morphologyEx(imgThresholded, cv2.MORPH_CLOSE, kernel2, iterations=4) # not better than canny 
    #_,contours, hierarchy = cv2.findContours(imgThresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #imgThresholded=cv2.drawContours(imgThresholded, contours, -1, (255,255,255), 3)
    cv2.imshow("canny_gold",imgThresholded)
    if imgThresholded==[]:
        print(" Error opening image\n")
        return -1;
    return imgThresholded


def image_modifications_silver(imgThresholded):
    # Taking a matrix of size 5 or 3 as the kernel 
    kernel = np.ones((5,5), np.uint8)
    kernel2 = np.ones((3,3), np.uint8) 

    imgThresholded=cv2.dilate(imgThresholded, kernel, iterations=7)#7) #high number to get rid of the little details on the coins
    #imgThresholded=cv2.erode(imgThresholded, kernel, iterations=2) #erode the white parts

    imgThresholded=cv2.dilate(imgThresholded, kernel2, iterations=2)#2) #dilate the white parts NOTE: if iteration 1 or 2 --> big difference over the circle result
    imgThresholded=cv2.erode(imgThresholded, kernel2, iterations=17)#17)
    cv2.imshow("canny_silver",imgThresholded)
    imgThresholded = cv2.Canny(imgThresholded,80,80)

    imgThresholded=cv2.dilate(imgThresholded, kernel, iterations=1) #dilate the white parts
    imgThresholded=cv2.erode(imgThresholded, kernel, iterations=1)

    #imgThresholded = cv2.morphologyEx(imgThresholded, cv2.MORPH_CLOSE, kernel2, iterations=4) #wanted to use it to close gaps, not better than canny 
    #_,contours, hierarchy = cv2.findContours(imgThresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #imgThresholded=cv2.drawContours(imgThresholded, contours, -1, (255,255,255), 3)
    if imgThresholded==[]:
        print(" Error opening image\n")
        return -1;
    return imgThresholded


def image_modifications(imgThresholded):
    #ret, imgThresholded = cv2.threshold(frame,127,255,cv2.THRESH_BINARY)
    #imgThresholded=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) #Convert the captured frame from BGR to GRAY
    #imgThresholded=cv2.medianBlur(imgThresholded, 15)
    #imgThresholded=cv2.blur(imgThresholded, (5,5))
    #imgThresholded=cv2.GaussianBlur(frame, (15,15),0)
    #ret, imgThresholded = cv2.threshold(imgThresholded, 127, 255, 0)
    #imgThresholded = cv2.adaptiveThreshold(imgThresholded, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV, 11, 1)


    # Taking a matrix of size 5 or 3as the kernel 
    kernel = np.ones((5,5), np.uint8)
    kernel2 = np.ones((3,3), np.uint8) 
    
  
    # close gaps in between object edges to get perfects circles
    imgThresholded=cv2.dilate(imgThresholded, kernel, iterations=1) #cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)) ) 
    imgThresholded=cv2.erode(imgThresholded, kernel, iterations=2) # erode the white parts
    imgThresholded=cv2.dilate(imgThresholded, kernel2, iterations=5) #dilate the white parts

    
    imgThresholded=cv2.erode(imgThresholded, kernel2, iterations=4)

    #imgThresholded = cv2.morphologyEx(imgThresholded, cv2.MORPH_CLOSE, kernel2, iterations=4) #dilation then erosion
    #imgThresholded=cv2.dilate(imgThresholded, kernel, iterations=1)
    #imgThresholded=cv2.erode(imgThresholded, kernel, iterations=1)
    imgThresholded = cv2.Canny(imgThresholded,80,80)
    cv2.imshow("my_color_is_black",imgThresholded)
    if imgThresholded==[]:
        print(" Error opening image\n")
        return -1;
    #_,contours, hierarchy = cv2.findContours(imgThresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #imgThresholded=cv2.drawContours(imgThresholded, contours, -1, (255,255,255), 3)
    
    return imgThresholded


 

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("usb_cam/image_raw/compressed",Image,self.callback)
    self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
    #self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)
  
  



  def callback(self,data):
    global count
    global old_distance
    global my_coins
    global my_matrix_check
    global height
    global index
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

#parameter of my computer's camera used to print the aruco axes
    #cameraMatrix =np.array([[611.029350, 0.000000, 291.500762],[0.000000, 608.520033, 236.523594],[0.000000, 0.000000, 1.000000]])
    #distCoeffs =np.array([[-0.028521,-0.110481, 0.001912, -0.009806, 0.000000]])

    Size_marker=40 #in mm
   
    corners, ids= aruco_detection(cv_image)
    frame=cv_image.copy()#in case no ids detected
    
    if ids != None : 
      #frame = aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)
      #cv2.imshow("checkfirst",frame)
      distance= calcul_distance(frame, corners,Size_marker)
      old_distance = distance
    if ids == None:
      distance =old_distance

    #if ids != None : 
      #rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.04, cameraMatrix, distCoeffs)
      #print ("rvecs")
      #print(rvecs)
    
    #frame= aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs, tvecs, 0.02);
    #imgThresholded=color_mask(frame)
    #cv2.imshow("testbbbbbb",imgThresholded)
    #imgThresholded= image_modifications(imgThresholded)

    
        
    check_copper(cv_image,distance)
    check_silver(cv_image,distance)
    check_gold(cv_image,distance)
    count+=1
    
    if count>24: #if camera moves a lot of coins will be registered at different positions, to avoid that reset my_coins to adapt to movement
        my_coins=[]
        my_matrix_check=[]
        index=0
        height=0
        count=0
    
    #cv2.imshow("ginal_window",cv_txt)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)



#This is the process that start once you press ctrl-C    
def handler(signum, frame):
  poslistx=[]
  poslisty=[]
  poslistv=[]

  if height!=0: # Check if there is something in my_matrix_check
    for ids in range(0,index):#for each coins registered
      positionx=my_coins[ids][1]
      positiony=my_coins[ids][2]
      poslistx.append(positionx)
      poslisty.append(positiony)
      sum=0
      count=0
      average=my_coins[ids][3] #so if height=0 i don't compare with the previous average
      for i in range(0,height): #we check the matrix with all the coin registered in that area
        if my_matrix_check[i][0]==ids: #if the index match (same area)
          sum+=my_matrix_check[i][3]
          count+=1

      try:
        average= sum/count #i do an average of all those numbers found in the same vicinty as the one stored in "my_coins"
        print("index")
        print(ids)
        print("average")
        print(average)
      except ZeroDivisionError: 
        pass
      if average !=my_coins[ids][3]: #if the average is different from my coin_value
        sum+=my_coins[ids][3]
        average=sum/(count+1)
      #if round(average)!=round(my_coins[ids][3]): #we check if the average match #not good because it only works if the diff is consequent
        my_coins[ids][3]=round(average)
        my_coins[ids][7]=average #then keep the average (which would be an estimation with the error taken into account)
        #another option would be to keep the number most seen.
      poslistv.append(my_coins[ids][3])         
  with open("/home/daria/my_ros_ws/src/image_processing/my_coin_test.txt", "w") as file:
    file.write(str("my_coins:[index,x,y,value of coin seen, size in mm, r in px, color_coin, average if there is an uncertainty]:    "))
    file.write(str(my_coins))
    file.write(str("               matrice_check:"))
    file.write(str(my_matrix_check)) #hopefully empty
  cv2.destroyAllWindows()
  #point_list= zip(poslistx,poslisty,poslistv)
  fig, ax = plt.subplots()
  ax.scatter(poslistx, poslisty)
  #rng = np.random.RandomState(0)
  #colors = rng.rand(100)
  #sizes = 1000 * rng.rand(100)
  #for i, ((x,y),) in enumerate(zip(point_list)):
  for i, txt in enumerate(poslistv):
        ax.annotate(txt, (poslistx[i], poslisty[i]))
        #plt.text(x,y,z, ha="center", va="center")

  #plt.scatter(poslistx,poslisty)    
  plt.show() #tried to display the result in a graph with a color bar to know the number
  """answer = input('We are almost done. Do you really want to exit? [yes]:')
  if answer == 'yes':
    print('bye')
    exit()
  print("Then let's keep running")"""

  

def main(args):
  signal.signal(signal.SIGINT, handler)
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except (KeyboardInterrupt, SystemExit):

    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
