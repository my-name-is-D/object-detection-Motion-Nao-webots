<div style="text-align:center">
<img src="https://github.com/sma20/HSR/blob/master/image/1200px-Heriot-Watt_University_logo.svg.png " width="150" >
</div>

# object detection/motion-nao webots
First step. Detect and extract object position, one version extracts object color. 

Second step. Send it to a computational model of rat with/without PD and extract relevant noise to send to the robot

Third step.choose where to send the noise that will perturb (or not) the robot motion, and execute an arm kinematic

Visualize it on webots, plot the motion with matplotlib

## Architecture

<div style="text-align:center">
<p align="center">
<img src="https://github.com/my-name-is-D/object-detection-Motion-Nao-webots/blob/master/img/process.PNG" width="400" >
</p>
</div>





## Image processing

Use find_2d_object & ROS or OpenCv & ROS (2 versions)

See objects, detect them, extract position in camera coordinates send the position and object ID (or color) as a topic (in cm in v1 and mm in v2). 
Either before or after a workspace filtering (if in send in topic, else it doesn't)
The workspace check is also effectuated here.

## kinematic nao

Either receive the object pose as a topic or from a txt file (must be in mm, the reference base is the centre of NAO's torso)
transform it into angle joint and send them back as a topic.
A version consider Nao's workspace before calculating the angle joints.
Another decompose the trajectory into sub-motion
The angle joints for each motion is saved in txt files. 

## rat_model

The rat model is jhielson's (see:https://github.com/jhielson) with adaptations to fit my case, see his github to install required dependencies
HAVE TO WAIT FOR THE AUTHORISATION TO PUBLISH WORK BASED ON JHIELSON'S IN MY GIT REP (Private for the moment)

The stimulation and Pd probabilities are saved in a csv file (in world/data)

However, all the machine learning tests are there, in a Machine Learning folder: Naive Bayes, Linear/Logistic regression, K-mean clusters, the support vector regression model and the feed forward ANN 
## world

Subscribe to the topic publsihed by kinematic nao and move nao to the given position.
The joint pose are recorded at given interval and stored in csv files. 
They can then be ploted later on with 
```bash
world/data/./plotdata
```
note:the simulated camera data is extracted and sent as a topic in a string format.

All the data are in this package.


See https://github.com/jhielson/neurorobotics_computational_environment for installing webots for this case. 
Note: don't hesitate to check the issue if you need to use gazebo on Ros melodic (gazebo 9) instead. (If you are on kinetic, there should be no issue normally) 
