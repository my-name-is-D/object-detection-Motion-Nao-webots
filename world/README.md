## in controller
controller-python.py 

The robot receive the joint angles to move to via a topic (from kinematicnao package) and execute it. 
The joint pose are mesured at a given interval (have yet to thread it) and written in a csv file. 

here you have info about how to control the motors, the time, create and manipulate motions (in closeopenhand and in the loop part)

Don't hesitate to ask if you don't understand something, or if something is uncomplete. 

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
