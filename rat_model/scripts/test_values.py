#! /usr/bin/env python
import rospy
#from std_msgs.msg import Float64
from rat_model.msg import interference
import random

def send_value():

    pub_int = rospy.Publisher('/interference', interference, queue_size=5)
    randvel=random.uniform(-1.01,8.26)
    randacc=random.uniform(-1.01,100)
    #randrange for an int and uniform for a float
    
    connections = pub_int.get_num_connections()
    if connections > 0:
        pub_int.publish([100, 0.1])
        #rospy.loginfo("Cmd Published")
    else:
        rospy.sleep(0.1)
    rospy.Rate(10).sleep()
    if connections > 0:
        pub_int.publish([0.5,0.1])
        #rospy.loginfo("Cmd Published")
    else:
        rospy.sleep(0.1)
    
    rospy.Rate(10).sleep()

    if connections > 0:
        pub_int.publish([70,4])
        #rospy.loginfo("Cmd Published")
    else:
        rospy.sleep(0.1)
    rospy.Rate(10).sleep()  

def main():
    rospy.init_node("muscle_interference")

    #Test unit 
    s=rospy.Rate(10)
    while not rospy.is_shutdown():
        send_value()
        s.sleep()
    

if __name__ == '__main__':
    main()