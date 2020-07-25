## in controller

### controller-python.py 

Only the left arm will move, reach and grasp 

The robot can receive the joint angles to move to via a topic (from kinematicnao package) and execute it. 
OR it can retrieve static joint poses from a file (anglejoint_static.txt in world/data) and exexute it.
There is a part in run to comment/uncomment to choose from one option or the other


The joint pose are mesured at a given interval and written in a csv file. 

This code subscribes to 2 topics: the left arm and right arm 

here you have info about how to control the motors, the time, create and manipulate motions (in closeopenhand and in the loop part)

Don't hesitate to ask if you don't understand something, or if something is uncomplete. 

to install :webots and nao 

see: - https://github.com/jhielson/neurorobotics_computational_environment

to launch everthing

```bash
roslaunch webots_ros complete.launch
```

### controller-python.py 
This code subscribes to 2 topics: the left arm and right arm 

The robot can receive the joint angles to move to via a topic (from kinematicnao package) and execute it. 
OR it can retrieve static joint poses from a file (anglejoint_static.txt in world/data) and exexute it.
There is a part in run to comment/uncomment to choose from one option or the other


The joint pose are mesured at a given interval and written in a csv file. 


here you have info about how to control the motors, the time, create and manipulate motions (in closeopenhand and in the loop part)

Don't hesitate to ask if you don't understand something, or if something is uncomplete. 


### controller_image.py
Same as above + Nao's head moves down and up and the bottom camera is read and the data is sent as a topic (gray image and blue image).
If a joint position is received, the head motion is interrupted to execute the arm motion.

Trick: the image is sent as a string, not as a multiarray. 



## In Motion
handLclose and HandLOpen.motion or shooT.motion (in your world, i erased it on this github) 

here you can see how to create your own motion. 

Warning: there is no motor feedback, however you can import a sensory feedback.

## In Data

The csv files are stored there (arm joint pose in time and phalanx joint pose in time)
a python file to plot them is also here. 
```bash
./plotdata.py
```


## Warnings
the proto may not be complete, see Webots official Nao's world and add my stuff to yours, it would be safer. 


