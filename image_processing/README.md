# What this package does
This package is composed of 2 versoins.
1. try to identify an image thanks to find_2d_object, process the object pose and extract the position according to the camera frame. Then the id of the object seen and its position x,y,z are sent as a topic. 
2. extract from a topic an image (sent as a string in the topic) reconstruct the string into an image format then extract ellipses from the picture and calculate the simulated ball position (known size).
If the average color of the ellipse is red (or non identified), nothing is sent back, else the color and position are sent back into a topic.

1
```bash
roslaunch image_processing launch.launch
```
2
```bash
rosrun image_processing simulation_image_form_color.py
```
or 
```bash
roslaunch kinematicnao mainworkspace(_decomposition).launch
```
# SETUP and INFO

## Using our camera:
```bash
sudo apt-get install ros-melodic-uvc-camera
rosrun uvc_camera uvc_camera_node
```
## To see our camera image: 
```bash
rqt_image_view
```

## Calibrating it :
- http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
(don't forget to print a SQUARED chestboard)

Note to calibrate: the created file will probably be in the temp files. You have to put it in ".ros" following the indication of your uvc_camera (saying when you launch it before calibrating "can't find calibration in path_to_follow")
After that you will probably have a new warning (or error) saying something like the names doesn't correspond or something.
Open the calibration file X.yaml and change the camera name to the one in [] presented in your terminal. Sorry no photo, didn't think of it on the spot.  

## We can then process it :
- http://wiki.ros.org/image_proc
```bash
rosrun image_proc image_proc (in my case)
```
Then we take as input 
/image_rect_color or /image_rect_color/compressed
instead of 
/image_raw (but well, i'm still using image_raw)


After calibration my focal lenght is more or less 605 (my fl in x and y aren't exactly the same)


## To detect objects:
Find_2d_object package (ROS)
```bash
apt-get install ros-melodic-find-object-2d
```

### The cpp files

I created the cpp files to extract the object position thanks to: 

- http://robotica.unileon.es/index.php/Objects_recognition_and_position_calculation_(webcam)
- https://husarion.com/tutorials/ros-tutorials/4-visual-object-recognition/

#### object_pose.cpp

The calculus are from me, the position is based on a camera referene, not NAO hand pose

NOTE: the distance is extracted thanks to the focallenght, the area of the object in px and the area of the object in mm, therefore using squared/rectangle/circle objects is better to calculate more easily the area. (my tube picture isn't giving good results, because i estimated wrongly the real area, while my box picture is giving fine results) 
The topic sent with this code is in cm

#### workspace_limit_object_pose.cpp

Same with the addition of a workspace limitation (in cm) based on a nao h25 v3.3
the position is sent only if it's in the workspace.
Also, the distance x is divided by 2, in order to have further points in the workspace (workspace on x of 21cm and my camera focus is of about 10cm)

### The python files

#### Simulation_image_form_color.py
This file subscribe to the topic sent by webots (the data is sent as a string because the Int32Multiarray didn't work but webots terminal didn't display errors, seriously if you can choose, gazebo is more practical if having a code adapted to a real situation is what you want, webots robots don't display topics... as would your robot).

the data is transformed into an opencv image, canny edge detection is applied and then a contour and ellipse detection is done (a circle one too, still in the code, but the ellipses give better results)
The position of a known ball (known size) is calculated 
Then, a part of the ellipse is prelevated and a color average is done, This color is then identified into: blue, yellow, green, red or none. 

Finally we verify if the ball is in Nao's workspace, a stimulation stimulus is sent according to the answe in a topic.
Then the color and position of the ball are sent in another topic.


#### Simulation_image_formcolorcylinder.py

It retrieves the same info from the same topics.
The object to detect is now a cylinder, therefore the visual processing strategy completely changed.
the bottom part of the cylinder is erased from the image considering that's the bottom will be darker than the top that receives the light (therefore the code is not adapted to dark video or to image lighten from behind the camera). Then a canny and ellipse detection is used.

The position of the cylinder is calculated and an offset is added (for the kinematic, to be able to grasp the object). Warning: the calculus obtained is ideal for a 10m radius, else need to add an offset (or redo a curve fitting).

The color average is the same (just note that the image is resized differently).
The workspace check is the same and the topic messages as well.


## Prerequisit

You need Opencv2 

