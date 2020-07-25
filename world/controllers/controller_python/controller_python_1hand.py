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

#Modified by my_name_is_D

import csv
import os
import numpy
import time
import rospy
from threading import Timer,Thread,Event

from controller import Motor, TouchSensor, PositionSensor
from controller import Robot, Motion

from std_msgs.msg import Float64, String, Int32MultiArray, Int8
from rat_model.msg import interference
#from rosgraph_msgs.msg import Clock
#--synchronize

#-----------------------NAO CLASS-------------------------#


class Nao (Robot,Motion,PositionSensor):
    PHALANX_MAX = 8
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
        #print(self.LElbowYaw.getMaxPosition()) 
        #print("data",data.data)
        #print("prev L",self.prev_Larmangles)
        if not ( data.data =="POSITION NOT REACHABLE"):
            #if self.Left_Resetallowed==True: #If the previous motion has been fully executed 
            self.Larmangles = data.data .split(',')
            for i in xrange(0,len(self.Larmangles)-1,4):
                if (float(self.Larmangles[i])> self.LShoulderPitch.getMaxPosition()):
                    self.Larmangles[i]=self.LShoulderPitch.getMaxPosition()
                elif (float(self.Larmangles[i])< self.LShoulderPitch.getMinPosition()):
                    self.Larmangles[i]=self.LShoulderPitch.getMinPosition()
                
                if (float(self.Larmangles[i+1])> self.LShoulderRoll.getMaxPosition()):
                    self.Larmangles[i+1]=self.LShoulderRoll.getMaxPosition()
                elif (float(self.Larmangles[i+1])< self.LShoulderRoll.getMinPosition()):
                    self.Larmangles[i+1]=self.LShoulderRoll.getMinPosition()    
                    
                if (float(self.Larmangles[i+2])> self.LElbowYaw.getMaxPosition()):
                    self.Larmangles[i+2]=self.LElbowYaw.getMaxPosition()
                elif (float(self.Larmangles[i+2])< self.LElbowYaw.getMinPosition()):
                    self.Larmangles[i+2]=self.LElbowYaw.getMinPosition()   
                
                if (float(self.Larmangles[i+3])> self.LElbowRoll.getMaxPosition()):
                    self.Larmangles[i+3]=self.LElbowRoll.getMaxPosition()
                elif (float(self.Larmangles[i+3])< self.LElbowRoll.getMinPosition()):
                    self.Larmangles[i+3]=self.LElbowRoll.getMinPosition()
                #if (self.Larmangles != self.prev_Larmangles):
                #    self.Left_Resetallowed=False 
                #print(self.Larmangles)    
        #else:
            
            #print(data.data)
            
  
        
    def rightjointcallback(self,data):
        """
        This function retrieve the angle of the robot (rad) from a topic send from kinematicnao and store them in an array.
        It also check if the angles are realisable. 
        """
        #print(self.LElbowYaw.getMaxPosition()) 
        if not ( data.data =="POSITION NOT REACHABLE"):
            #if self.Right_Resetallowed==True: #If the previous motion has been fully executed 
            self.Rarmangles = data.data .split(',')
            for i in xrange(0,len(self.Rarmangles)-1,4):
                if (float(self.Rarmangles[i])> self.RShoulderPitch.getMaxPosition()):
                    self.Rarmangles[i]=self.RShoulderPitch.getMaxPosition()
                elif (float(self.Rarmangles[i])< self.RShoulderPitch.getMinPosition()):
                    self.Rarmangles[i]=self.RShoulderPitch.getMinPosition()
                
                if (float(self.Rarmangles[i+1])> self.RShoulderRoll.getMaxPosition()):
                    self.Rarmangles[i+1]=self.RShoulderRoll.getMaxPosition()
                elif (float(self.Rarmangles[i+1])< self.RShoulderRoll.getMinPosition()):
                    self.Rarmangles[i+1]=self.RShoulderRoll.getMinPosition()    
                    
                if (float(self.Rarmangles[i+2])> self.RElbowYaw.getMaxPosition()):
                    self.Rarmangles[i+2]=self.RElbowYaw.getMaxPosition()
                elif (float(self.Rarmangles[i+2])< self.RElbowYaw.getMinPosition()):
                    self.Rarmangles[i+2]=self.RElbowYaw.getMinPosition()   
                
                if (float(self.Rarmangles[i+3])> self.RElbowRoll.getMaxPosition()):
                    self.Rarmangles[i+3]=self.RElbowRoll.getMaxPosition()
                elif (float(self.Rarmangles[i+3])< self.RElbowRoll.getMinPosition()):
                    self.Rarmangles[i+3]=self.RElbowRoll.getMinPosition() 
                #if (self.Rarmangles != self.prev_Rarmangles):
                #    self.Right_Resetallowed=False  
            print(self.Rarmangles) 
                    #self.Right_Resetallowed=False   
        #else:
            
            #print(data.data)
      
    def interferencecallback(self,data):
        """
        Test function to control the motion speed through a topic. Not on use 
        """
        self.musclesvelocity=data.interference[1]
        self.musclesacc= data.interference[0]
        print("self.musclesvelocity and acc",self.musclesvelocity, self.musclesacc)
          
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

    def printCameraImage(self, camera):
        """
        retrieve the data from the simulated camera and publish it as a topic 
        (in a string because at this point i didn't succeed in using a personnal msg from another package, didn't change it then)
        """
        scaled = 1  # defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # read rgb pixel values from the camera
        image = camera.getImage()
        
        #print('----------camera image (gray levels)---------')
        #print('original resolution: %d x %d, scaled to %d x %f'
        #      % (width, height, width / scaled, height / scaled))

        #self.image_matrix_gray = [0 for x in range(0,(width // scaled)* (height // scaled))] 
        #self.image_matrix_blue = [0 for x in range(0,(width // scaled)* (height // scaled))]  
                #images matrix
        image_matrix_blue = [] #The color image
        image_matrix_gray = [] # The gray scaled image
        
        for y in range(0, height // scaled):
            line = ''
            for x in range(0, width // scaled):
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled)
               
                #print(len(image_matrix_gray))
                #print( len(image_matrix_gray[0]))
                
                red = camera.imageGetRed(image, width, x * scaled, y * scaled) 
                blue = camera.imageGetBlue(image, width, x * scaled, y * scaled) 
                green= camera.imageGetGreen(image, width, x * scaled, y * scaled) 
                #line = line + str(int(gray))+ ' '

                image_matrix_gray.append(str(gray))

                image_matrix_blue.append(str(blue))
                image_matrix_blue.append(str(green))
                image_matrix_blue.append(str(red))
                #self.image_matrix_gray[y*(width // scaled)+x]=gray
                #self.image_matrix_blue[y*(width // scaled)+x]=blue
            #print(line)
        
        str1 = " " 
        
        #For some reasons the publisher doesn't work with Int32Multiarray... but no error displayed. So there i cheated a bit (other option: use a home made msg from another pkg)
        image_string_gray=str1.join(image_matrix_gray)
        image_string_blue=str1.join(image_matrix_blue)

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

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            self.threadgetvalues()
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
        
        #Hand touch sensor
        #self.HandTouchLeft = self.getTouchSensor('LHand/Touch/Right')
        
        #ARM SENSORS 
        self.RShoulderPitchS = self.getPositionSensor("RShoulderPitchS")
        self.LShoulderPitchS = self.getPositionSensor("LShoulderPitchS")      
        self.LShoulderRollS = self.getPositionSensor("LShoulderRollS")
        self.RShoulderRollS = self.getPositionSensor("RShoulderRollS")
        self.LElbowYawS = self.getPositionSensor("LElbowYawS")
        self.RElbowYawS = self.getPositionSensor("RElbowYawS")
        self.LElbowRollS = self.getPositionSensor("LElbowRollS")
        self.RElbowRollS = self.getPositionSensor("RElbowRollS")
        self.LWristYawS = self.getPositionSensor("LWristYawS")
        self.RWristYawS = self.getPositionSensor("RWristYawS")

        
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
        
    def move_with_callback(self):
        """
        ARM motion WITH CALLBACK()
        """
        while (self.Larmangles==[]):# and self.Rarmangles==[]):
            continue
        
        if (self.prev_Larmangles!=self.Larmangles):
            self.setHandsAngle(0.96)
            #robot.step(10)# let's give a bit of time (warry with the thread get value)
            
            #Both arms are simetric so we can do them at once with Larmangles
            for i in xrange(0,len(self.Larmangles)-1,4):
                """
                #Close hand at the last sub-motion
                if i >= len(self.Larmangles)-5:
                    self.setHandsAngle(0.00)
                """
                #print(len(self.Larmangles))
                #print("\n IIIIIIIII",i)
                self.threadgetvalues()
               
                
 #-----------------------------LEFT ARM-----------------------------------------
                self.LShoulderPitch.setPosition(float(self.Larmangles[i]))
                self.LShoulderPitch.setVelocity(self.musclesvelocity)
                self.LShoulderPitch.setAcceleration(self.musclesacc)
                self.LShoulderRoll.setPosition(float(self.Larmangles[i+1]))
                self.LShoulderRoll.setVelocity(self.musclesvelocity)
                self.LShoulderRoll.setAcceleration(self.musclesacc)
                self.LElbowYaw.setPosition(float(self.Larmangles[i+2]))
                self.LElbowYaw.setVelocity(self.musclesvelocity)
                self.LElbowYaw.setAcceleration(self.musclesacc)
                self.LElbowRoll.setPosition(float(self.Larmangles[i+3]))
                self.LElbowRoll.setVelocity(self.musclesvelocity)
                self.LElbowRoll.setAcceleration(self.musclesacc)
                #print("shoulder pitch R and wanted:",self.LShoulderPitchS.getValue(),float(self.Larmangles[i]))
"""
 #-----------------------------RIGHT ARM-----------------------------------------
                self.RShoulderPitch.setPosition(float(self.Rarmangles[i]))
                self.RShoulderPitch.setVelocity(self.musclesvelocity)
                self.RShoulderPitch.setAcceleration(self.musclesacc)
                self.RShoulderRoll.setPosition(float(self.Rarmangles[i+1]))
                self.RShoulderRoll.setVelocity(self.musclesvelocity)
                self.RShoulderRoll.setAcceleration(self.musclesacc)
                self.RElbowYaw.setPosition(float(self.Rarmangles[i+2]))
                self.RElbowYaw.setVelocity(self.musclesvelocity)
                self.RElbowYaw.setAcceleration(self.musclesacc)
                self.RElbowRoll.setPosition(float(self.Rarmangles[i+3]))
                self.RElbowRoll.setVelocity(self.musclesvelocity)
                self.RElbowRoll.setAcceleration(self.musclesacc)
                
"""                
                
                #Only for left, as we consider Left and right to have a somewhat simetric motion, it may truncate motion of the right arm 
                while (abs(self.LShoulderPitchS.getValue()-float(self.Larmangles[i]))>0.1 or abs(self.LShoulderRollS.getValue()-float(self.Larmangles[i+1]))>0.1 or abs(self.LElbowYawS.getValue()-float(self.Larmangles[i+2]))>0.1 or  abs(self.LElbowRollS.getValue()-float(self.Larmangles[i+3]))>0.1):
                    robot.step(2)
                    #print(" ")#savior print... 
                    """
                    if (abs(self.LShoulderPitchS.getValue()-float(self.Larmangles[i]))>0.1):
                        print("Lshoulderpitch",float(self.Larmangles[i]), self.LShoulderPitchS.getValue())
                    if abs(self.LShoulderRollS.getValue()-float(self.Larmangles[i+1]))>0.1:
                        print("LshoulderRoll",float(self.Larmangles[i+1]), self.LShoulderRollS.getValue())
                    if abs(self.LElbowYawS.getValue()-float(self.Larmangles[i+2]))>0.1:
                        print("LElbowYawS",float(self.Larmangles[i+2]), self.LElbowYawS.getValue())
                    if abs(self.LElbowRollS.getValue()-float(self.Larmangles[i+3]))>0.1:
                        print("LElbowRollS",float(self.Larmangles[i+3]), self.LElbowRollS.getValue())
                    """
                    #print("shoulder pitch R and wanted:",self.LShoulderPitchS.getValue(),float(self.Larmangles[i]))
                    #continue
                #print("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII",i)
                #print("LLLEEEEEEEEEEEEEEEEEENNNNNGGGGTTTHHHHHHHHHH",len(self.Larmangles)-1)
                print("end motion")
                self.Left_Resetallowed=True
                self.Right_Resetallowed=True
                self.prev_Larmangles=self.Larmangles
                #self.prev_Rarmangles=self.Rarmangles

    def move_with_file(self):
	"""	
	ARM motion WITH READANGLEPOSE(), when not using the camera nor the topics, we can retrieve the joints angle from a file
	"""
        Larmangles=self.readanglepose()
        self.LShoulderPitch.setPosition(float(Larmangles[0][0]))
        #self.LShoulderPitch.setVelocity(2)
        #self.LShoulderPitch.setAcceleration(10)
 
        self.LShoulderRoll.setPosition(float(Larmangles[0][1]))
        self.LElbowYaw.setPosition(float(Larmangles[0][2]))
        self.LElbowRoll.setPosition(float(Larmangles[0][3]))

    def readanglepose(self):
        """
        When not using the camera nor the topic, retrieve the joint angle from this file (filled by kinematic nao filemain.cpp) -file to recreate-
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
             
    def __init__(self):
    
        Robot.__init__(self)
              
        # initialize stuff
        self.currentlyPlaying = False
        self.jointsensor=[] #to obtain muscle pos
        self.handsensor=[]
        self.Larmangles=[] #to obtain muscle angle from callback LEFT ARM
        self.Rarmangles=[] #to obtain muscle angle from callback RIGHT ARM
        self.prev_Larmangles=[] #to check if there is been a new muscle order
        self.prev_Rarmangles=[]
        self.sample=0 #to know how many time we sampled the sensors
        self.musclesvelocity=2
        self.musclesacc=30
        self.timestep= self.TIMESTEP#int(self.getBasicTimeStep())
        self.Left_Resetallowed=True #to reset the joint callback after a full motion execution (avoid to do it before)
        self.Right_Resetallowed=True
        self.findAndEnableDevices()
        self.loadMotionFiles()
        
        #Publish camera feedback
        self.image_gray_pub = rospy.Publisher('gray_image',  String , queue_size=10)
        self.image_blue_pub = rospy.Publisher('blue_image',  String , queue_size=10)
        
        rospy.Subscriber("rightjointangles",String, self.rightjointcallback)#To get the rad of the motion RIGHT ARM
        rospy.Subscriber("jointangles",String, self.jointcallback)#To get the rad of the motion LEFT ARM
        rospy.Subscriber("/interference",interference,self.interferencecallback) #to modify the speed /acc of the motion -not used-
        #MyThread class initialization
        self.stopFlag = Event()
        self.thread = MyThread(self.stopFlag)
        self.thread.start()
        #threading.Thread(target=self.threadgetvalues).start() #test
        
    def run(self):
	
        rate = rospy.Rate(0.1) 
                
        while robot.step(self.timestep) != -1 and not rospy.is_shutdown():
            
            self.printCameraImage(self.cameraBottom)
            
            #we wait for the arm to get to its starting pose before doing the rest.
            self.move_with_callback()          
            	    
            self.printCameraImage(self.cameraBottom)

            #to check any change and break the head motion (not to wait 2sec)
            prev_Larmangles=self.Larmangles
            #self.setHandsAngle(0.96)

            #Head goes up and down, registering and sending image;
            while ( abs((self.HeadPitchS.getValue()-self.HeadPitch.getMaxPosition()))>0.1 or self.HeadPitchS.getValue()<0):
                #print ("down",self.HeadPitchS.getValue())
                #print("check:", abs((self.HeadPitchS.getValue()-self.HeadPitch.getMaxPosition())))
                self.HeadPitch.setPosition(self.HeadPitch.getMaxPosition())
                self.HeadPitch.setVelocity(0.3)
                #self.HeadPitch.setAcceleration(0.2) 
                self.threadgetvalues()
                robot.step(20)
                self.printCameraImage(self.cameraBottom)
                self.move_with_callback()
                #if (self.Larmangles!=prev_Larmangles):
                #    self.setHandsAngle(0.96)
                #    break

            #The motion is repeated after each while just not to lose time if there is been a ball seen;
            self.move_with_callback()
            self.writearmfile()
            self.writehandfile()
            
            prev_Larmangles=self.Larmangles
            while (abs((self.HeadPitchS.getValue()+0.4))>0.1 or self.HeadPitchS.getValue()>0):
                self.HeadPitch.setPosition(-0.4)
                #print ("up",self.HeadPitchS.getValue())
                self.HeadPitch.setVelocity(0.3)
                #self.HeadPitch.setAcceleration(0.2) 
                self.printCameraImage(self.cameraBottom)   
                self.threadgetvalues()  
                robot.step(20)  
                self.move_with_callback()
                
                
                self.writearmfile() 
                self.writehandfile()        
                #robot.step(75)
                #if (prev_Larmangles!=self.Larmangles):
                #    self.setHandsAngle(0.96)
                #    break
            
            self.printCameraImage(self.cameraBottom)
            #self.setAllLedsColor(0xff0000)#to check if launched corectly
                        
            #self.setHandsAngle(0.00)
            #self.setHandsAngle(0.00)
         
            #ARM motion WITH CALLBACK()
            self.move_with_callback()
            
            """
            #ARM motion WITH READANGLEPOSE()
            self.move_with_file()
            """
            
            
            #print(self.RShoulderPitchS.getType())#result: nan
            #self.closeAndOpenHands(2)#test closeopen hand
             
            self.setAllLedsColor(0x0000) 
            
            self.writearmfile()
            self.writehandfile()
            #self.printCameraImage(self.cameraBottom)

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

if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

if __name__ == '__main__':
    robot = Nao()
    robot.run()
    # this will stop the timer
    self.stopFlag.set()

    # Webots triggered termination detected!
    #saveExperimentData() #doesn't work
