# object detection/motion-nao webots
first step. Detect and extract object position. 

## Image processing

Use find_2d_object, ROS

See objects, detect them, extract position in camera coordinates send the position and object ID as a topic. 

## kinematic nao

either receive the object pose as a topic or from a txt file (must be in mm, the reference base is on NAO's feet)
transform it into angle joint and send them back as a topic

## world

Subscribe to the topic publsihed by kinematic nao and move nao to the given position.
The joint pose are recorded at given interval and stored in csv files. 
They can then be ploted later on. 
