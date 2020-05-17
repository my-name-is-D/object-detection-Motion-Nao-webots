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
from controller import Motor, TouchSensor, PositionSensor

from controller import Robot, Keyboard, Motion
import rospy
from std_msgs.msg import Float64, String

#from rosgraph_msgs.msg import Clock
#--synchronize

class Nao (Robot,Motion,PositionSensor):
    PHALANX_MAX = 8
    MOTOR_SPEED= 5 #rad/s
    TIMESTEP = 50 #wait time
    #def getTime(self):
    """
    def isOver(self):
        return self.Motion_isOver(self)
    """
    # load motion files
    
    
    
    def threadgetvalues(self):
        #print(self.getTime())
        self.sample=self.sample+1
        self.jointsensor.append([self.sample,self.getTime(),round(self.LShoulderPitchS.getValue(),3),round(self.LShoulderRollS.getValue(),3),
        round(self.LElbowYawS.getValue(),3),
        round(self.LElbowRollS.getValue(),3),
        round(self.LWristYawS.getValue(),3)])
        
        
        for i in range(0, self.PHALANX_MAX):
            self.handsensor.append([self.sample,self.getTime(),round(self.lphalanxsensor[i].getValue(),3)])
            
       
        #print(self.handsensor)
        """
        for joint in range(0, len(self.armjoints)/2-1):
            self.file.append([self.getTime(),self.armjoint[joint].getValue()])
        """ 
        
    def jointcallback(self,data):
         
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
      
        
    def loadMotionFiles(self):
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
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion


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
        
    def printGps(self):
        p = self.gps.getValues()
        print('----------gps----------')
        print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))

    # the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print('----------inertial unit----------')
        print('roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2]))

    def printFootSensors(self):
        fsv = []  # force sensor values

        fsv.append(self.fsr[0].getValues())
        fsv.append(self.fsr[1].getValues())

        l = []
        r = []

        newtonsLeft = 0
        newtonsRight = 0

        # The coefficients were calibrated against the real
        # robot so as to obtain realistic sensor values.
        l.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Front Left
        l.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Front Right
        l.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Rear Right
        l.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Rear Left

        r.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Front Left
        r.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Front Right
        r.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Rear Right
        r.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Rear Left

        for i in range(0, len(l)):
            l[i] = max(min(l[i], 25), 0)
            r[i] = max(min(r[i], 25), 0)
            newtonsLeft += l[i]
            newtonsRight += r[i]

        print('----------foot sensors----------')
        print('+ left ---- right +')
        print('+-------+ +-------+')
        print('|' + str(round(l[0], 1)) +
              '  ' + str(round(l[1], 1)) +
              '| |' + str(round(r[0], 1)) +
              '  ' + str(round(r[1], 1)) +
              '|  front')
        print('| ----- | | ----- |')
        print('|' + str(round(l[3], 1)) +
              '  ' + str(round(l[2], 1)) +
              '| |' + str(round(r[3], 1)) +
              '  ' + str(round(r[2], 1)) +
              '|  back')
        print('+-------+ +-------+')
        print('total: %f Newtons, %f kilograms'
              % ((newtonsLeft + newtonsRight), ((newtonsLeft + newtonsRight) / 9.81)))

    def printFootBumpers(self):
        ll = self.lfootlbumper.getValue()
        lr = self.lfootrbumper.getValue()
        rl = self.rfootlbumper.getValue()
        rr = self.rfootrbumper.getValue()
        print('----------foot bumpers----------')
        print('+ left ------ right +')
        print('+--------+ +--------+')
        print('|' + str(ll) + '  ' + str(lr) + '| |' + str(rl) + '  ' + str(rr) + '|')
        print('|        | |        |')
        print('|        | |        |')
        print('+--------+ +--------+')

    def printUltrasoundSensors(self):
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print('-----ultrasound sensors-----')
        print('left: %f m, right %f m' % (dist[0], dist[1]))

    def printCameraImage(self, camera):
        scaled = 2  # defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # read rgb pixel values from the camera
        image = camera.getImage()

        print('----------camera image (gray levels)---------')
        print('original resolution: %d x %d, scaled to %d x %f'
              % (width, height, width / scaled, height / scaled))

        for y in range(0, height / scaled):
            line = ''
            for x in range(0, width / scaled):
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255  # rescale between 0 and 9
                line = line + str(int(gray))
            print(line)

    def setAllLedsColor(self, rgb):
        # these leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # ear leds are single color (blue)
        # and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)
        
 ########################################################################################
 ########################################################################"
 
    def closeAndOpenHands(self, countSpikes):
        for i in range(countSpikes):
            
            #rospy.get_rostime() #get time as rospy.Time instance
            #rospy.get_time() #get time as float secs
            #rospy.sleep(1) 
            #self.startMotion(self.OpenLHand)
            
            """
            while self.OpenLHand.isOver() == False:
                robot.step(self.timeStep)
            """
		
            #print("duration")
            #print (self.OpenLHand.getDuration())
            
            #print(self.getTargetPosition())
            
            
            self.setHandsAngle(0.96)
            
            robot.step(200)
            self.setHandsAngle(0.00)
            robot.step(200)
            
            
            """
            self.startMotion(self.CloseLHand)
            while self.CloseLHand.isOver() == False:
                robot.step(self.timeStep)
            """
                #robot.getPosition()
            #robot.step(200)
            #rospy.sleep(1)
            """
            print("DURATION")
            print(motion.getDuration())
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
                #cCCCCCCCprint(self.lphalanx[i].getAcceleration())
            if len(self.lphalanx) > i and self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)
                #self.lphalanx[i].setVelocity(10)
                #self.lphalanx[i].setAcceleration(10)


    def printHelp(self):
        print('----------nao_demo_python----------')
        print('Use the keyboard to control the robots (one at a time)')
        print('(The 3D window need to be focused)')
        print('[Up][Down]: move one step forward/backwards')
        print('[<-][->]: side step left/right')
        print('[Shift] + [<-][->]: turn left/right')
        print('[U]: print ultrasound sensors')
        print('[A]: print accelerometers')
        print('[G]: print gyros')
        print('[S]: print gps')
        print('[I]: print inertial unit (roll/pitch/yaw)')
        print('[F]: print foot sensors')
        print('[B]: print foot bumpers')
        print('[Home][End]: print scaled top/bottom camera image')
        print('[PageUp][PageDown]: open/close hands')
        print('[7][8][9]: change all leds RGB color')
        print('[0]: turn all leds off')
        print('[H]: print this help message')

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

        # gps
        self.gps = self.getGPS('gps')
        self.gps.enable(4 * self.timeStep)

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
        
        #SENSORS
        self.armjointsensors=[]
        armjoints=['LShoulderPitchS',"LShoulderRollS","LElbowYawS","LElbowRollS","LWristYawS"
                        'RShoulderPitchS',"RShoulderRollS","RElbowYawS","RElbowRollS", "RWristYawS"]    
        
        """"
        for i in range(0,len(armjoints)/2-1):
        # shoulder pitch sensors
            self.armjointsensors.append(self.getPositionSensor(armjoints[i]))
            self.armjointsensors[i].enable(self.timeStep)
        """
        
        
       
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
        
        # shoulder roll motors
        self.RShoulderRoll = self.getMotor("RShoulderRoll")
        self.LShoulderRoll = self.getMotor("LShoulderRoll")
        
        # Elbow yaw motors    
        self.RElbowYaw = self.getMotor("RElbowYaw")
        self.LElbowYaw = self.getMotor("LElbowYaw")
        # Elbow Roll motors    
        self.RElbowRoll = self.getMotor("RElbowRoll")
        self.LElbowRoll = self.getMotor("LElbowRoll")
        
        # Elbow Roll motors    
        self.RWristYaw = self.getMotor("RWristYaw")
        self.LWristYaw= self.getMotor("LWristYaw")
        
        
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
        
        # Elbow Roll motors    
        self.RWristYaw = self.getMotor("RWristYaw")
        self.LWristYaw= self.getMotor("LWristYaw")
        
        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)
    
    def writehandfile(self):
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
        
            
            
        """
        with open(paths,'w') as csvFile:
            f = open("../../data/test.txt", "a")
            f.write(self.file)
            f.write("\n")
            f.close()
        """
    def writeobjectpose(self):
        file = open("../../data/write.txt", "w")
        #for left arm, lolipop pose
        file.write(str(0.101001)+"\n")
        file.write(str(0.312183)+"\n")
        file.write(str(0.122544)+"\n")
        #print str(target[2]) + " , " +str(target[0]) + " , " + str(-target[1])
        file.close()
    
    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False
        self.jointsensor=[] #to obtain muscle pos
        self.handsensor=[]
        self.Larmangles=[] #to obtain muscle angle from callback
        self.sample=0
        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.timestep= self.TIMESTEP#int(self.getBasicTimeStep())
        #self.printHelp()   
        
        #threading.Thread(target=self.threadgetvalues).start()
        self.pub = rospy.Publisher('motor', Float64, queue_size=10)
        #rospy.Subscriber("spikes", String, self.callback) 
        rospy.Subscriber("jointangles",String, self.jointcallback)
    def readanglepose(self):
        Larmangles=[]
        print("in")
        while (Larmangles == []):
            with open("/home/cata/nao_ws/src/NAOKinematics/NaoPythonIK/cpp/read.txt","r") as file2:
                for line in file2:
                    #file2 = open("/home/cata/nao_ws/src/NAOKinematics/NaoPythonIK/cpp/read.txt","r")
                    List = [elt.strip() for elt in line.split(',')]
                    Larmangles.append(List)
                    #print(file2.read().split(',')) 
            
            print(Larmangles)
        return Larmangles
        
        
        
        
        
    def callback(self, data):
        print("I heard %s", data.data)
        count = (int)(data.data)
        self.closeAndOpenHands(count)
        
    def run(self):
        #self.handWave.setLoop(True)
        #self.handWave.play()

        # until a key is pressed
        #key = -1
        #while robot.step(self.timeStep) != -1:
        #    key = self.keyboard.getKey()
        #    if key > 0:
        #        break
        #self.handWave.setLoop(False)

        #while True:
        rate = rospy.Rate(0.1) 
        self.writeobjectpose()
        
        while robot.step(self.timestep) != -1 and not rospy.is_shutdown():
                      
            #Larmangles=self.readanglepose()
            self.setAllLedsColor(0xff0000)
            """
            print("before")
            print(self.getTime())
            print(self.RShoulderPitchS.getValue())
            """
            
            self.threadgetvalues()
            #self.setHandsAngle(0.96)
            
            while (self.Larmangles==[]):
                continue
                
            #WITH CALLBACK()
            self.LShoulderPitch.setPosition(float(self.Larmangles[0]))
            self.LShoulderRoll.setPosition(float(self.Larmangles[1]))
            self.LElbowYaw.setPosition(float(self.Larmangles[2]))
            self.LElbowRoll.setPosition(float(self.Larmangles[3]))
            
            """
            #WITH READANGLEPOSE()
            self.LShoulderPitch.setPosition(float(Larmangles[0][0]))
            self.LShoulderRoll.setPosition(float(Larmangles[0][1]))
            self.LElbowYaw.setPosition(float(Larmangles[0][2]))
            self.LElbowRoll.setPosition(float(Larmangles[0][3]))
            """
            robot.step(20)
            
            #self.setHandsAngle(0.00)
            
            #print(self.RShoulderPitchS.getType

            """
            robot.step(50)
            print(self.getTime())
            print(self.RShoulderPitchS.getValue())
            print(self.RShoulderPitchS.getType())
            """
            #print(self.RWristYaw.getMinPosition())
            #print(self.RWristYaw.getMaxPosition())
            #self.closeAndOpenHands(2)
            self.threadgetvalues()
            robot.step(20)
            self.threadgetvalues()
            robot.step(20)
            self.threadgetvalues()
            robot.step(200)
            self.threadgetvalues()
            
            #rospy.sleep(0.1)  
            self.setAllLedsColor(0x0000) 
            
            self.writearmfile()
            
            #self.writearmfile()
            #self.printCameraImage(self.cameraBottom)
            """
            now= self.getTime()
            print(now)
            """
	 # pulish simulation clock
            """
            msg = Clock()
            time = robot.getTime()
            msg.clock.secs = int(time)
            # round prevents precision issues that can cause problems with ROS timers
            msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
            clockPublisher.publish(msg)
            """

           

            #open and read the file after the appending:
            #f = open("demofile2.txt", "r")
            #print(f.read()) 
            #break
            if robot.step(self.timeStep) == -1:
                break


# Webots

# ROS
rospy.init_node('controller', anonymous=True)
# we want to use simulation time for ROS
#clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
#CameraPublisher = rospy.Publisher('cameradata',
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')


robot = Nao()
robot.run()

# Webots triggered termination detected!
#saveExperimentData()
