#walking algo v1 for dog project

#main UI interpreter

import numpy as np 
import math as m

dawg = 0

class Leg():
    __hipPos__ = 0
    __shoulderPos__ = 0
    __kneePos__ = 0

    hipRange = []
    shoulderRange = []
    kneeRange = []

    def __init__(self):
        self.hipPos = 0
        self.shoulderPos = 0
        self.kneePos = 0
        #hip motor ranges, assuming that its a 0-360 increase TODO: test min/max vals on robot
        self.hipRange = [0,60]
        self.shoulderRange = [0,180]
            #real positional angle value, actual motor values calculated 
        self.kneeRange = [60,120]

    #used when reading from arduino for real motor pos feedback
    def updateVals(self, hip, shoulder, knee):
        self.hipPos = hip
        self.shoulderPos = shoulder
        self.kneePos = knee

    #should only be given positive values
    def ratio(self, a, b):
        print(f"a: {a}")
        print(f"b: {b}")
        
        a = float(a)
        b = float(b)

        if a>b:
            return b/a
        if b>a:
            return a/b
        
    
        #passing in self, and leg to compare it to
        #compares all joints in a leg to all joints in another, eventually outputting a compare ratio based on percentage deviation 
    def compare(self, other):
        hip = self.__hipPos__
        shoulder = self.__shoulderPos__
        knee = self.__kneePos__
        hip1 = other.getHip()
        shoulder1 = other.getShoulder()
        knee1 = other.getKnee()
        r1 = self.ratio(hip, hip1)
        r2 = self.ratio(shoulder, shoulder1)
        r3 = self.ratio(knee, knee1)
        print(f"ratio: {r1}")
        print(f"ratio: {r2}")
        print(f"ratio: {r3}")
        return ((r1+r2+r3)/3)


    #individual setters for writing to motor vals (use for setting init vals, etc)

    def setHip(self, hip):
        hipPos = hip
    
    def setShoulder(self, should):
        shoulderPos = should

    def setKnee(self, knee):
        kneePos = knee

    #getters for alg
    def getHip(self):
        return self.hipPos

    def getShoulder(self):
        return self.shoulderPos

    def getKnee(self):
        return self.kneePos
    

#runs only when a new move command is sent, sends a position and a desired movement speed
def send(p, speedFactor):
    output = []
    output.append(speedFactor, p.getRF().getHip(), p.getRF().getShoulder(), p.getRF().getKnee(), p.lFHip, p.lFShoulder, p.lFKnee, p.rBHip, p.rBShoulder, p.rBKnee, p.lBHip, p.lBShoulder, p.lBKnee)
    print(str(output))
    #string format (sends raw motor positional values for the onboard PID to move the robot to):
    #speedFactor, rFHip, rFShoulder, rFKnee, lFHip, lFShoulder, lFKnee, rBHip, rBShoulder, rBKnee, lBHip, lBShoulder, lBKnee

    #TODO: function to send from raspi to arduino here
    return


class Position():
    __rF__ = 0
    __lF__ = 0
    __rB__ = 0
    __lB__ = 0
    #where are the legs touching the ground?
    __contactPoints__ = []

    def __init__(self):
        self.__rF__ = Leg()
        self.__lF__ = Leg()
        self.__rB__ = Leg()
        self.__lB__ = Leg()

    def updateVals(self, rFHip, rFShoulder, rFKnee, lFHip, lFShoulder, lFKnee, rBHip, rBShoulder, rBKnee, lBHip, lBShoulder, lBKnee):
        print(f"inputarr: {[rFHip, rFShoulder, rFKnee, lFHip, lFShoulder, lFKnee, rBHip, rBShoulder, rBKnee, lBHip, lBShoulder, lBKnee]}")
        #updates current motor position values in each leg
        self.__rF__.updateVals(rFHip, rFShoulder, rFKnee)
        self.__lF__.updateVals(lFHip, lFShoulder, lFKnee)
        self.__rB__.updateVals(rBHip, rBShoulder, rBKnee)
        self.__lB__.updateVals(lBHip, lBShoulder, lBKnee)

    def getRF(self):
        return self.__rF__
    def getLF(self):
        return self.__lF__
    def getRB(self):
        return self.__rB__
    def getLB(self):
        return self.__lB__
    
    
    #returns a number from zero to 1 representing percentage accuracy to compareTo position
    def compareTo(self, compareTo):
        r1 = self.getRF().compare(compareTo.getRF())
        r2 = self.getLF().compare(compareTo.getLF())
        r3 = self.getRB().compare(compareTo.getRB())
        r4 = self.getLB().compare(compareTo.getLB())
        print(r1)
        print(r2)
        print(r3)
        print(r4)
        return ((r1+r2+r3+r4)/4)

class MyPoint3D():
    x = 0
    y = 0
    z = 0
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

#handles the gait geometry generation, inverse kinematics, etc
class Gait():
    __goalPos__ = 0
    __tipPos__ = []
    __posArr__ = []
    robot = 0

    def __init__(self, robot):
        self.robot = robot

    def generate(self, arr):
        self.__tipPos__ = arr
        #generate an array of angular positions for each leg based on the input array of "step frames"
        for tipSets in self.__tipPos__:
            p = Position()
            rfTip = tipSets[0]
            lfTip = tipSets[1]
            rbTip = tipSets[2]
            lbTip = tipSets[3]

            

            #TODO: take the set of 4 leg tips and convert them each to joint angles, then update values accordingly

            p.updateVals(10,10,10,10,10,10,10,10,10,10,10,10)
            self.__posArr__.append(p)
        self.__goalPos__ = self.__posArr__[0]
        return 
    
    def nextPos(self):
        #checks to make sure index of current goal position is not last index in __posArr__
        if self.__posArr__.index(self.__goalPos__) < (len(self.__posArr__)-1):
            #given that it isn't the last one, increase index value by 1
            self.__goalPos__ = self.__posArr__[self.__posArr__.index(self.__goalPos__)+1]
        #if __goalPos__ is last index in array, go back to begining of loop
        else:
            self.__goalPos__ = self.__posArr__[0]
        return self.__goalPos__
    
    def setGoalPos(self, p):
        self.__goalPos__ = p
    def getGoalPos(self):
        return self.__goalPos__
    def getPosArr(self):
        return self.__posArr__


class MotorControl():
    accuracyVal = 0
    walkF = 0
    rotatR = 0
    stand = 0
    liftTest = 0
    def __init__(self):
        #acceptable proportion of accuracy of current val/desired val NOTE: current val is the real feedback value from motors, post c++ PID functions
        self.accuracyVal = 0.9

    def initialize(self, robot):
        self.robot = robot

        #creates various gaits based on an array of 4 3d points per position, with relation to a (0,0) at the center of the robot
        #TODO: create a seperate function/program to generate step profiles algorithmically, but use this for now:

        #walkF
        self.walkF = Gait(self.robot)
        arr = [0]
        #first "frame" of gait
        #TODO: adjust for real distance values in meters
        rf = MyPoint3D(10,5,0)
        lf = MyPoint3D(-10,5,0)
        rb = MyPoint3D(10,-5,0)
        lb = MyPoint3D(-10,-5,0)
        tips = [rf, lf, rb, lb]
        arr[0] = tips
        #second "frame"
        rf = MyPoint3D(10,5,0)
        lf = MyPoint3D(-10,5,0)
        rb = MyPoint3D(10,-5,0)
        lb = MyPoint3D(-10,-5,0)
        tips = [rf, lf, rb, lb]
        arr.append(tips)
        #creates the list of positions corresponding to the foot coordinates 
        self.walkF.generate(arr)

        #rotatR
        self.rotatR = Gait(self.robot)
        arr = [0]
        #first "frame" of gait
        rf = MyPoint3D(10,5,0)
        lf = MyPoint3D(-10,5,0)
        rb = MyPoint3D(10,-5,0)
        lb = MyPoint3D(-10,-5,0)
        tips = [rf, lf, rb, lb]
        arr[0] = tips
        #creates the list of positions corresponding to the foot coordinates 
        self.rotatR.generate(arr)

        #liftTest
        self.liftTest = Gait(self.robot)
        arr = [0]

        #stand function, should be default when dog is on
        self.stand = Gait(self.robot)
        arr = [0]
        rf = MyPoint3D(10,5,0)
        lf = MyPoint3D(-10,5,0)
        rb = MyPoint3D(10,-5,0)
        lb = MyPoint3D(-10,-5,0)
        tips = [rf, lf, rb, lb]
        arr[0] = tips
        self.stand.generate(arr)

    #takes movement gait type, and speed coeff (0-1), then looks to see if robot has reached desired location, controlling rate of movement commands to arduino
    def go(self, type, speed):
        gait = 0
        if type == "walkF":
            gait = self.walkF
        elif type == "rotate":
            gait = self.rotatR
        elif type == "stand":
            gait = self.stand
        else:
            print("Dog().MotorController().go() err, invalid gait type")
            return

        #if the robot's current position is sufficiently close to the last step, then move to the next step
        if self.robot.getPos().compareTo(gait.getGoalPos())>self.accuracyVal:
            self.robot.moveTo(gait.nextPos(), speed)


class Dog():

    currpos = 0
    controller = 0

    hipRange = 50
    shoulderRange = 180
    kneeRange = 90

    tibiaLength = 0
    femurLength = 0
    hipDist = 0
    shoulderDist = 0
    kneeDist = 0


    def __init__(self):
        self.currpos = Position() 
        self.controller = MotorControl()
        #all in meters, the physical parameters of the robot

        #distance from shoulder to knee
        self.femurLength = 0.237 
        #distance from knee to foot
        self.tibiaLength = 0.54 
        #distance laterally "out" from hip axis to shoulder joint (same yz plane as foot)
        self.hipDist = 0.055 
        #distance diagonally up between line going through center of femur and parallel line through shoulder axis 
        self.shoulderDist = 0.023 
        #distance diagonally down between line going through center of tibia and parallel line through tip of foot 
        self.kneeDist = 0


        #TODO: configure hardware, write convert function
        #returns joint angles based on motor values
    def adjustFromReality(self, val, pos):
        #rFHip = Right front hip motor position
        if pos == 4:

            return val
        #rFShoulder = Right front shoulder motor position
        elif pos == 5:

            return val
        #rFKnee = Right front knee motor position
        elif pos == 6:

            return val
        #lFHip = Left front hip motor position
        elif pos == 7:

            return val
        #lFShoulder = Left front shoulder motor position
        elif pos == 8:

            return val
        #lFKnee = Left front knee motor position
        elif pos == 9:

            return val
        #rBHip = Right rear hip motor position
        elif pos == 10:

            return val
        #rBShoulder = Right rear shoulder motor position
        elif pos == 11:

            return val
        #rBKnee = Right rear knee motor position
        elif pos == 12:

            return val
        #lBHip = Left rear hip motor position
        elif pos == 13:

            return val
        #lBShoulder = Left rear shoulder motor position
        elif pos == 14:

            return  val
         #lBKnee = Left rear knee motor position
        elif pos == 15:

            return val

    #returns real adjusted motor values when given joint angles
    #also handles the scaling due to belt reductions at the knee, etc
    def adjustToReality(p):
        #TODO: configure hardware, code adjustment function
        return p
    
    #sends the final move command to arduino
    def moveTo(self, p, speed):
        position = p
        send(self.adjustToReality(p), speed)
        return

    def getPos(self):
        return self.currpos
    
    #initializes motion controller 
    def initialize(self):
        self.controller.initialize(self)
        
def main():
    dawg = Dog()
    dawg.initialize()
    #sending a dummy string from "arduino"
    testcmd = "1, 0, false, false, 30, 30, 90, 30, 30, 90, 30, 30, 90, 30, 30, 90"
    read(dawg, testcmd)
    testcmd = "1, 0, false, false, 31, 32, 90, 29, 32, 90, 31, 32, 90, 29, 32, 90"
    read(dawg, testcmd)
    return

#runs as much as possible, continually reading live position data sent from onboard arduino and updating dawg.currpos accordingly
def read(dog, command):
    #string format:
    #rsY, lsX, isbut1, isbut2, rFHip, rFShoulder, rFKnee, lFHip, lFShoulder, lFKnee, rBHip, rBShoulder, rBKnee, lBHip, lBShoulder, lBKnee

    #rsY = float right stick y val
    #lsX = float left stick x val
    #isbut1 = boolean value for if button 1 is being pressed
    #isbut1 = boolean value for if button 2 is being pressed
    #rFHip = Right front hip motor position
    #rFShoulder = Right front shoulder motor position
    #rFKnee = Right front knee motor position
    #lFHip = Left front hip motor position
    #lFShoulder = Left front shoulder motor position
    #lFKnee = Left front knee motor position
    #rBHip = Right rear hip motor position
    #rBShoulder = Right rear shoulder motor position
    #rBKnee = Right rear knee motor position
    #lBHip = Left rear hip motor position
    #lBShoulder = Left rear shoulder motor position
    #lBKnee = Left rear knee motor position

    #gets command as a string, parses it into an array
    cmd = list(command.split(','))
    print(f"cmd: {cmd}")

    #converts appropriate array values to integers
    for i in range(4,16):
        cmd[i] = int(cmd[i])
        dog.adjustFromReality(cmd[i], i)

    #updates the dog's stored realtime position based on input motor data from arduino
    dog.getPos().updateVals(cmd[4], cmd[5], cmd[6], cmd[7], cmd[8], cmd[9], cmd[10], cmd[11], cmd[12], cmd[13], cmd[14], cmd[15])

    #modified control structure so can only perform 1 distinct operation at once TODO: algo gradient merging multiple gaits?
    if cmd[0] != 0:
        if float(cmd[0])>0:
            dog.controller.go("walkF", 1)
    elif cmd[1] != 0:
        dog.controller.go("rotate", 1)
    elif bool(cmd[2]):
        print("butt1  press")
        dog.controller.go("stand", 1)
    elif bool(cmd[3]):
        print("butt2  press")
        #what to do if button 2 is being pressed
        

if __name__ == '__main__':
    main()