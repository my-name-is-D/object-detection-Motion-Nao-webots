## NAO Kinematic on ROS

# original work
- https://github.com/noobswestand/NaoPythonIK
(himself inspired from Kouretes github, see thesis :https://www.cs.umd.edu/~nkofinas/Projects/KofinasThesis.pdf for understanding NAO kinematic)

# How this package work
It has been adapted to ROS.
 
 ### V1
 An object position and orientation can be written in a txt file and read here, 
 the kinematic find the appropriate angles to reach the point and send as a topic "jointangles" 
 the angles from the shoulder to the elbow to reach the end pose. 
```bash
roslaunch kinematicnao filekinematicnao.launch 
```
The txt file can be found in the folder data of this package, but in the code the file read is in the "world" 
package of this repository

### V2
Another version of the code has been realised. 

The object position is retrieved from a topic. 
The topic is created by the image_processing package (launch.launch)
then the rest is the same

```bash
roslaunch kinematicnao kinematicnao.launch 
```

### V3

The object position is retrieved from a file.
If the position is in Nao's left arm workspace then the inverse kinematic is done, else a "position not ok" (or something) is sent through the topic "jointangles"

```bash
roslaunch kinematicnao workspacefilenao.launch 
```
Note: the workspace is a simplified version of Nao's workspace


The workspace_stimulation.py is the python version of the workspace limitation, it send a binary stimulation if the point is in or out of the workspace
