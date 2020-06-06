# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 

"""Example of Python controller for Nao robot.
   This demonstrates how to access sensors and actuators"""


import csv
import os
import numpy
import time
import rospy
from threading import Timer,Thread,Event

from controller import Motor, TouchSensor, PositionSensor
from controller import Robot, Keyboard, Motion
from std_msgs.msg import Float64, String, Int32MultiArray, Int8

#from rosgraph_msgs.msg import Clock
#--synchronize


#-----------------------NAO CLASS-------------------------#


class Nao (Robot,Motion,PositionSensor):
    PHALANX_MAX = 8
    MOTOR_SPEED= 5 #rad/s
    TIMESTEP = 50 #wait time
    
    
    
    
    """
    This function is called from another class, the MyThread class. It is to be called at given interval.
    This method may work on the real Nao (to test) but not in the simulation. 
    Observation: the period of prelevement isn't periodic. Often the values are taken before the movement and after, not during (or 1 at most)
    Hypothesis :Nao is only one instance in the program. So if you send order to move you can't retrieve the result of the order at the same time, you have to wait for the 
    first order to have been successfully given. once it is, high chance the full movement has already been executed. 
    New observation: without robot.step() the sample are taken more periodically
     
    """
    
    @classmethod
    def threadgetvalues(cls):
        """
        This function retrieve the sensors of the muscles values and store them in an array (appartening to NAO class)
        """
        #print(robot.getTime())
        robot.sample=robot.sample+1
        robot.jointsensor.append([robot.sample,robot.getTime(),round(robot.LShoulderPitchS.getValue(),3),round(robot.LShoulderRollS.getValue(),3),
        round(robot.LElbowYawS.getValue(),3),
        round(robot.LElbowRollS.getValue(),3),
        round(robot.LWristYawS.getValue(),3)])
        #print(robot.jointsensor)
        
        for i in range(0, robot.PHALANX_MAX):
            robot.handsensor.append([robot.sample,robot.getTime(),round(robot.lphalanxsensor[i].getValue(),3)])     
        #print(self.handsensor)
  
        
    def jointcallback(self,data):
        """
        This function retrieve the angle of the robot (rad) from a topic send from kinematicnao and store them in an array.
        It also check if the angles are realisable. 
        """
         
        if not ( data.data =="POSITION NOT REACHABLE"):
            self.Larmangles = data.data .split(',')
            
            if (float(self.Larmangles[0])> self.LShoulderPitch.getMaxPosition()):
                self.Larmangles[0]=self.LShoulderPitch.getMaxPosition()
            elif (float(self.Larmangles[0])< self.LShoulderPitch.getMinPosition()):
                self.Larmangles[0]=self.LShoulderPitch.getMinPosition()
            
            if (float(self.Larmangles[1])> self.LShoulderRoll.getMaxPosition()):
                self.Larmangles[1]=self.LShoulderRoll.getMaxPosition()
            elif (float(self.Larmangles[1])< self.LShoulderRoll.getMinPosition()):
                self.Larmangles[1]=self.LShoulderRoll.getMinPosition()    
                
            if (float(self.Larmangles[2])> self.LElbowYaw.getMaxPosition()):
                self.Larmangles[2]=self.LElbowYaw.getMaxPosition()
            elif (float(self.Larmangles[2])< self.LElbowYaw.getMinPosition()):
                self.Larmangles[2]=self.LElbowYaw.getMinPosition()   
            
            if (float(self.Larmangles[3])> self.LElbowRoll.getMaxPosition()):
                self.Larmangles[3]=self.LElbowRoll.getMaxPosition()
            elif (float(self.Larmangles[3])< self.LElbowRoll.getMinPosition()):
                self.Larmangles[3]=self.LElbowRoll.getMinPosition()       
            print(self.Larmangles)    
        else:
            
            print(data.data)
      
        
    def loadMotionFiles(self):
        """
        You can send pre-determined motions to the simulated nao, here you retrieve those files
        """
        self.handWave = Motion('../../motions/HandWave.motion')
        self.forwards = Motion('../../motions/Forwards50.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')
        self.CloseLHand = Motion('../../motions/CloseLHand.motion')
        self.OpenLHand = Motion('../../motions/OpenLHand.motion')
        
        
    def startMotion(self, motion):
        """
        To start the motions stored in the file (read the file)
        """
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

#---------------------PRINT DATA (useless
    # the accelerometer axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print('----------accelerometer----------')
        print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))

    # the gyro axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printGyro(self):
        vel = self.gyro.getValues()
        print('----------gyro----------')
        # z value is meaningless due to the orientation of the Gyro
        print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))

    # the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print('----------inertial unit----------')
        print('roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2]))

    def printUltrasoundSensors(self):
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print('-----ultrasound sensors-----')
        print('left: %f m, right %f m' % (dist[0], dist[1]))

    def printCameraImage(self, camera):
        scaled = 1  # defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # read rgb pixel values from the camera
        image = camera.getImage()
        
        print('----------camera image (gray levels)---------')
        print('original resolution: %d x %d, scaled to %d x %f'
              % (width, height, width / scaled, height / scaled))

        #self.image_matrix_gray = [0 for x in range(0,(width // scaled)* (height // scaled))] 
        #self.image_matrix_blue = [0 for x in range(0,(width // scaled)* (height // scaled))]  
                #images matrix
        image_matrix_blue = []
        image_matrix_gray = []   
        
        for y in range(0, height // scaled):
            line = ''
            for x in range(0, width // scaled):
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled)  # rescale between 0 and 9
               
                #print(len(image_matrix_gray))
                #print( len(image_matrix_gray[0]))
                
                red = camera.imageGetRed(image, width, x * scaled, y * scaled) # rescale between 0 and 9
                blue = camera.imageGetBlue(image, width, x * scaled, y * scaled) 
                green= camera.imageGetGreen(image, width, x * scaled, y * scaled) 
                line = line + str(int(gray))+ ' '
                image_matrix_gray.append(str(gray))
                image_matrix_blue.append(str(blue))
                image_matrix_blue.append(str(green))
                image_matrix_blue.append(str(red))
                #self.image_matrix_gray[y*(width // scaled)+x]=gray
                #self.image_matrix_blue[y*(width // scaled)+x]=blue
            #print(line)
        
        str1 = " " 
        
        #For some reasons the publisher doesn't work with Int32Multiarray... but no error displayed. So there i cheated a bit
        image_string_gray=str1.join(image_matrix_gray)
        image_string_blue=str1.join(image_matrix_blue)
        #print(image_string_blue)

        self.image_gray_pub.publish(image_string_gray)
        self.image_blue_pub.publish(image_string_blue)
           
            
    def setAllLedsColor(self, rgb):
        # these leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # ear leds are single color (blue)
        # and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)
        
        
 ########################## FUNCTION CLOSE OPEN HAND ###################
 
    def closeAndOpenHands(self, countSpikes):
        for i in range(countSpikes):
            
            """
            #TEST with the motions files
            
            self.startMotion(self.OpenLHand)
            while self.OpenLHand.isOver() == False:
                robot.step(self.timeStep)
	
            print("duration")
            print (self.OpenLHand.getDuration())
            print(self.getTargetPosition())
            """
            
            self.setHandsAngle(0.96)
            robot.step(200)
            self.setHandsAngle(0.00)
            robot.step(200)

            """
            #TEST with the motions files
            self.startMotion(self.CloseLHand)
            while self.CloseLHand.isOver() == False:
                robot.step(self.timeStep)
            """
            print('X%d' %(i))
                     

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            clampedAngle = angle
            if clampedAngle > self.maxPhalanxMotorPosition[i]:
                clampedAngle = self.maxPhalanxMotorPosition[i]
            elif clampedAngle < self.minPhalanxMotorPosition[i]:
                clampedAngle = self.minPhalanxMotorPosition[i]

            if len(self.rphalanx) > i and self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
                #print(self.rphalanx[i].getAcceleration())
                #print(self.rphalanx[i].getVelocity())
                
                #self.lphalanx[i].setVelocity(0.5)
                #print(self.lphalanx[i].getAcceleration())
            if len(self.lphalanx) > i and self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)
                #self.lphalanx[i].setVelocity(10)
                #self.lphalanx[i].setAcceleration(10)


    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = self.TIMESTEP#int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getCamera("CameraTop")
        self.cameraBottom = self.getCamera("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # accelerometer
        self.accelerometer = self.getAccelerometer('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # gyro
        self.gyro = self.getGyro('gyro')
        self.gyro.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getInertialUnit('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ['Sonar/Left', 'Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDistanceSensor(usNames[i]))
            self.us[i].enable(self.timeStep)
        
        #Hand touch sensor
        #self.HandTouchLeft = self.getTouchSensor('LHand/Touch/Right')
        
        #ARM SENSORS       
        self.LShoulderRollS = self.getPositionSensor("LShoulderRollS")
        self.RShoulderRollS = self.getPositionSensor("RShoulderRollS")
        self.LElbowYawS = self.getPositionSensor("LElbowYawS")
        self.RElbowYawS = self.getPositionSensor("RElbowYawS")
        self.LElbowRollS = self.getPositionSensor("LElbowRollS")
        self.RElbowRollS = self.getPositionSensor("RElbowRollS")
        self.LWristYawS = self.getPositionSensor("LWristYawS")
        self.RWristYawS = self.getPositionSensor("RWristYawS")
        self.RShoulderPitchS = self.getPositionSensor("RShoulderPitchS")
        self.LShoulderPitchS = self.getPositionSensor("LShoulderPitchS")
        
        self.RShoulderPitchS.enable(self.timeStep)
        self.LShoulderPitchS.enable(self.timeStep)
        self.LShoulderRollS.enable(self.timeStep)
        self.RShoulderRollS.enable(self.timeStep) 
        self.LElbowYawS.enable(self.timeStep)
        self.RElbowYawS.enable(self.timeStep)
        self.LElbowRollS.enable(self.timeStep)
        self.RElbowRollS.enable(self.timeStep)
        self.LWristYawS.enable(self.timeStep)
        self.RWristYawS.enable(self.timeStep)
        

        """
        # foot sensors
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getTouchSensor(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # foot bumpers
        self.lfootlbumper = self.getTouchSensor('LFoot/Bumper/Left')
        self.lfootrbumper = self.getTouchSensor('LFoot/Bumper/Right')
        self.rfootlbumper = self.getTouchSensor('RFoot/Bumper/Left')
        self.rfootrbumper = self.getTouchSensor('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)
        """
        
        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getLED('ChestBoard/Led'))
        self.leds.append(self.getLED('RFoot/Led'))
        self.leds.append(self.getLED('LFoot/Led'))
        self.leds.append(self.getLED('Face/Led/Right'))
        self.leds.append(self.getLED('Face/Led/Left'))
        self.leds.append(self.getLED('Ears/Led/Right'))
        self.leds.append(self.getLED('Ears/Led/Left'))

        # get phalanx motor tags
        # the real Nao has only 2 motors for RHand/LHand
        # but in Webots we must implement RHand/LHand with 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.lphalanxsensor=[]
        self.rphalanxsensor=[]
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getMotor("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getMotor("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())
            
            self.lphalanxsensor.append(self.getPositionSensor("LPhalanx%dS" % (i + 1)))
            self.rphalanxsensor.append(self.getPositionSensor("RPhalanx%dS" % (i + 1)))
            self.lphalanxsensor[i].enable(self.timeStep)
            self.rphalanxsensor[i].enable(self.timeStep)

        self.HeadPitch = self.getMotor("HeadPitch")
        self.HeadPitchS = self.getPositionSensor("HeadPitchS")
        
        self.HeadPitchS.enable(self.timeStep)
        # shoulder pitch motors
        self.RShoulderPitch = self.getMotor("RShoulderPitch")
        self.LShoulderPitch = self.getMotor("LShoulderPitch")
        # shoulder roll motors
        self.RShoulderRoll = self.getMotor("RShoulderRoll")
        self.LShoulderRoll = self.getMotor("LShoulderRoll")
        
        # Elbow yaw motors    
        self.RElbowYaw = self.getMotor("RElbowYaw")
        self.LElbowYaw = self.getMotor("LElbowYaw")
        # Elbow Roll motors    
        self.RElbowRoll = self.getMotor("RElbowRoll")
        self.LElbowRoll = self.getMotor("LElbowRoll")
        
        # Wrist Yaw motors    
        self.RWristYaw = self.getMotor("RWristYaw")
        self.LWristYaw= self.getMotor("LWristYaw")
   
        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)
    
    
    def writehandfile(self):
        """
        Write in the hand.csv file the joint pose over time
        """
        my_path=os.path.abspath(os.path.dirname(__file__))
        paths=os.path.join(my_path,"../../data/hand.csv")
        with open(paths,'w') as csvFile:
        
            field=['timestamp','LPhalanx1','LPhalanx2','LPhalanx3','LPhalanx4','LPhalanx5','LPhalanx6','LPhalanx7','LPhalanx8']
            writer=csv.DictWriter(csvFile,fieldnames=field)
            writer.writeheader()  
            for i in range(0, len(self.handsensor),8):
            
                  writer.writerow({'timestamp':self.handsensor[i][1],
                'LPhalanx1':self.handsensor[i][2] ,'LPhalanx2': self.handsensor[i+1][2],'LPhalanx3': self.handsensor[i+2][2],
                'LPhalanx4': self.handsensor[i+3][2],
                'LPhalanx5': self.handsensor[i+4][2],'LPhalanx6': self.handsensor[i+5][2],'LPhalanx7': self.handsensor[i+6][2],
                'LPhalanx8':self.handsensor[i+7][2]})
    
    def writearmfile(self):
        """
        Write in the arm.csv file the joint pose over time
        """
        #open the csv file in which to write
        my_path=os.path.abspath(os.path.dirname(__file__))
        paths=os.path.join(my_path,"../../data/arm.csv")
        with open(paths,'w') as csvFile:
        
            field=['timestamp','LshoulderP','LshoulderR','LelbowY','LelbowR','LwristY']
            writer=csv.DictWriter(csvFile,fieldnames=field)
            writer.writeheader()  
            
            for i in range(0,len(self.jointsensor)):
                writer.writerow({'timestamp':self.jointsensor[i][1],
                'LshoulderP':self.jointsensor[i][2] ,'LshoulderR': self.jointsensor[i][3],'LelbowY': self.jointsensor[i][4],
                'LelbowR': self.jointsensor[i][5],
                'LwristY': self.jointsensor[i][6]})
        
         
    def writeobjectpose(self):
        """
        Test function. 
        Write in a file the end pose to reach (x,y,z rx,ry,rz) (to send to kinematicnao)
        This is when we don't use the camera nor the topics
        """
        file = open("../../data/objectpoint.txt", "w")
        #for left arm, lolipop pose
        file.write(str(0.101001)+"\n")
        file.write(str(0.312183)+"\n")
        file.write(str(0.122544)+"\n")
        file.write(str(0.0)+"\n")
        file.write(str(0.0)+"\n")
        file.write(str(0.0)+"\n")
        #print str(target[2]) + " , " +str(target[0]) + " , " + str(-target[1])
        file.close()
    
    def __init__(self):
    
        Robot.__init__(self)
              
        #rospy.Timer(rospy.Duration(1), self.getsensorvalues)
        self.currentlyPlaying = False
        self.jointsensor=[] #to obtain muscle pos
        self.handsensor=[]
        self.Larmangles=[] #to obtain muscle angle from callback
        self.sample=0 #to know how many time we sampled the sensors
        
        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.timestep= self.TIMESTEP#int(self.getBasicTimeStep())
        #self.printHelp()   
         

        #threading.Thread(target=self.threadgetvalues).start() #test
        self.pub = rospy.Publisher('motor', Float64, queue_size=10)
        #rospy.Subscriber("spikes", String, self.callback) 
        
        #Publish camera feedback
        self.image_gray_pub = rospy.Publisher('gray_image',  String , queue_size=10)
        self.image_blue_pub = rospy.Publisher('blue_image',  String , queue_size=10)
        
        
        rospy.Subscriber("jointangles",String, self.jointcallback)#o get the rad of the motion
        #MyThread class initialization 
        self.stopFlag = Event()
        self.thread = MyThread(self.stopFlag)
        self.thread.start()
        
        
    def readanglepose(self):
        """
        When not using the camera nor the topic, retrieve the joint angle from this file (filled by kinematic nao filemain.cpp)
        """
        Larmangles=[]
        #print("in")
        while (Larmangles == []):
            with open("/home/cata/nao_ws/src/world/data/anglejoint_static.txt","r") as file2:
                for line in file2:
                    List = [elt.strip() for elt in line.split(',')]
                    Larmangles.append(List)
            #print(Larmangles)
        return Larmangles
          
        
    def callback(self, data):
        """
        This function receive a number and open/close teh hand this number of time.
        """
        print("I heard %s", data.data)
        count = (int)(data.data)
        self.closeAndOpenHands(count)
        
    def run(self):
        rate = rospy.Rate(0.1) 
        #self.writeobjectpose()
        
        while robot.step(self.timestep) != -1 and not rospy.is_shutdown():
            
            #print ("begin",self.HeadPitchS.getValue())
            self.printCameraImage(self.cameraBottom)
            #Head goes up and down, registering and sending image;
            
            while ( abs((self.HeadPitchS.getValue()-self.HeadPitch.getMaxPosition()))>0.1 or self.HeadPitchS.getValue()<0):
                #print ("down",self.HeadPitchS.getValue())
                #print("check:", abs((self.HeadPitchS.getValue()-self.HeadPitch.getMaxPosition())))
                self.HeadPitch.setPosition(self.HeadPitch.getMaxPosition())
                self.HeadPitch.setVelocity(0.1)
                #self.HeadPitch.setAcceleration(0.2) 
                #self.printCameraImage(self.cameraBottom) 
                robot.step(20)
                self.printCameraImage(self.cameraBottom)
            
            while (abs((self.HeadPitchS.getValue()+0.4))>0.1 or self.HeadPitchS.getValue()>0):
                self.HeadPitch.setPosition(-0.4)
                #print ("up",self.HeadPitchS.getValue())
                self.HeadPitch.setVelocity(0.1)
                #self.HeadPitch.setAcceleration(0.2) 
                #self.printCameraImage(self.cameraBottom)     
                robot.step(20)           
                #robot.step(75)
                self.printCameraImage(self.cameraBottom)
            
            
            self.printCameraImage(self.cameraBottom)
                
            self.setAllLedsColor(0xff0000)#to check if launched corectly
                        
            #self.threadgetvalues()
            self.setHandsAngle(0.96)
            #robot.step(30)
            
         
            #ARM motion WITH CALLBACK()
            
            while (self.Larmangles==[]):
                continue
            
            self.LShoulderPitch.setPosition(float(self.Larmangles[0]))
            self.LShoulderPitch.setVelocity(4)
            self.LShoulderPitch.setAcceleration(50)
            self.LShoulderRoll.setPosition(float(self.Larmangles[1]))
            self.LShoulderRoll.setVelocity(4)
            self.LShoulderRoll.setAcceleration(self.TIMESTEP)
            self.LElbowYaw.setPosition(float(self.Larmangles[2]))
            self.LElbowYaw.setVelocity(4)
            self.LElbowYaw.setAcceleration(self.TIMESTEP)
            self.LElbowRoll.setPosition(float(self.Larmangles[3]))
            self.LElbowRoll.setVelocity(4)
            self.LElbowRoll.setAcceleration(self.TIMESTEP)
            
            """
            #ARM motion WITH READANGLEPOSE()
            Larmangles=self.readanglepose()
            self.LShoulderPitch.setPosition(float(Larmangles[0][0]))
            #self.LShoulderPitch.setVelocity(2)
            #self.LShoulderPitch.setAcceleration(10)
 
            self.LShoulderRoll.setPosition(float(Larmangles[0][1]))
            self.LElbowYaw.setPosition(float(Larmangles[0][2]))
            self.LElbowRoll.setPosition(float(Larmangles[0][3]))
            """
            #robot.step(400)
            #self.LShoulderPitch.setPosition(-1.6)
            #self.setHandsAngle(0.00)
            
            #print(self.RShoulderPitchS.getType())#result: nan

            #self.closeAndOpenHands(2)#test closeopen hand
            """
            self.threadgetvalues()
            robot.step(20)
            self.threadgetvalues()
            robot.step(20)
            self.threadgetvalues()
            robot.step(200)
            self.threadgetvalues()
            """
             
            self.setAllLedsColor(0x0000) 
            
            self.writearmfile()
            #self.writehandfile()
            #self.printCameraImage(self.cameraBottom)

           
	 # pulish ROS simulation clock -not quite succesfull-
            """
            msg = Clock()
            time = robot.getTime()
            msg.clock.secs = int(time)
            # round prevents precision issues that can cause problems with ROS timers
            msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
            clockPublisher.publish(msg)
            """

            #break
            if robot.step(self.timeStep) == -1:
                # this will stop the timer
                self.stopFlag.set()
                break

"""
This class is used to call threadgetvalue at a period time
"""
class MyThread(Thread, Nao):
    def __init__(self, event,):
        Thread.__init__(self)
        self.stopped = event
        
    def run(self):
        while not self.stopped.wait(0.003): #Set periodic time.
            #print("my thread")
            
            Nao.threadgetvalues()
      
      

# ROS
rospy.init_node('controller', anonymous=True)

# we want to use simulation time for ROS
#clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
#CameraPublisher = rospy.Publisher('cameradata',
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

if __name__ == '__main__':
    robot = Nao()
    robot.run()
    # this will stop the timer
    self.stopFlag.set()

    # Webots triggered termination detected!
    #saveExperimentData()
