# SETUP and INFO

## Using our camera:
sudo apt-get install ros-melodic-uvc-camera
rosrun uvc_camera uvc_camera_node

## To see our camera image: 
rqt_image_view

## Calibrating it :
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
(don't forget to print a squared chestboard)

## We can then process it :
http://wiki.ros.org/image_proc

rosrun image_proc image_proc (in my case)

Then we take as input 
/image_rect_color or /image_rect_color/compressed
instead of 
/image_raw (but well, i'm still using image_raw)


After calibration my focal lenght is more or less 605 (my fl in x and y aren't exactly the same)


## To detect objects:
FIND_2d_object package (ROS)

### The cpp file
I created the cpp file to extract the object position thanks to:
http://robotica.unileon.es/index.php/Objects_recognition_and_position_calculation_(webcam)
https://husarion.com/tutorials/ros-tutorials/4-visual-object-recognition/

NOTE: the distance is extracted thanks to the focallenght, the area of the object in px and the area of the object in mm, therefore using squared/rectangle/circle objects is better to calculate more easily the area. (my tube picture isn't giving good results, because i estimated wrongly the real area, while my box picture is giving fine results) 

### The python files
The two python files are codes previously made (by me or my team) to extract features with opencv (the "*good*.py") 
Recognize money and their value. Done with pounds (NB: this file here is not executable as it is, i cleaned it a bit, will put the real code later, but not in this repository)
The marker*.py extract info from find_2D_object and send them to RVIZ, the rviz part doesn't work anymore right now (not been readapted)
The marker*.py is from : https://github.com/my-name-is-D/Robotic-System-Science-Project




