#!/usr/bin/env python3

from __future__ import division
import time
import math
from collections import deque
import os

########################################################################
##
## File I/O functions
##
########################################################################

# Function for fast reading from sensor files
def FastRead(infile):
    infile.seek(0)    
    value = int(infile.read().decode().strip())
    return(value)

# Function for fast writing to motor files    
def FastWrite(outfile,value):
    outfile.truncate(0)
    outfile.write(str(int(value)))
    outfile.flush()    

########################################################################
##
## Sensor Setup
##
########################################################################

# Open sensor files for (fast) reading
touchSensorValueRaw = open("ev3devices/in1/value0", "rb")
gyroPitchRaw  = open("ev3devices/in2/value0", "rb")
gyroRolllRaw  = open("ev3devices/in3/value0", "rb")
irSensor  = open("ev3devices/in4/value0", "rb")

# Initial state of the touch sensor
touchSensorPressed = FastRead(touchSensorValueRaw)

#pitch (forward positive) gyro in2
#roll (to left positive) gyro in3

# Set gyro to rate mode
with open('ev3devices/in2/mode', 'w') as f:
    f.write('GYRO-RATE')   
with open('ev3devices/in3/mode', 'w') as f:
    f.write('GYRO-RATE')

# Set Remote control mode
with open('ev3devices/in4/mode', 'w') as f:
    f.write('IR-REMOTE')

# Touch sensor macros
def WaitForTouchPress():
    touchSensorPressed = FastRead(touchSensorValueRaw)  
    while not touchSensorPressed: 
        touchSensorPressed = FastRead(touchSensorValueRaw) 
        time.sleep(0.1)

def WaitForTouchRelease():
    touchSensorPressed = FastRead(touchSensorValueRaw) 
    while touchSensorPressed: 
        touchSensorPressed = FastRead(touchSensorValueRaw) 
        time.sleep(0.1) 

########################################################################
##
## Motor Setup
##
########################################################################

#Roll  1 A, normal dir
#Pitch 1 B, normal dir
#Roll  2 C, reversed
#Pitch 2 D, reversed

with open('ev3devices/outC/polarity', 'w') as f:
    f.write('inversed')       
with open('ev3devices/outD/polarity', 'w') as f:
    f.write('inversed')     

# Open sensor files for (fast) reading
motorEncoderRolll1 = open("ev3devices/outA/position", "rb")   
motorEncoderPitch1 = open("ev3devices/outB/position", "rb")   
motorEncoderRolll2 = open("ev3devices/outC/position", "rb")   
motorEncoderPitch2 = open("ev3devices/outD/position", "rb")    
     
# Open motor files for (fast) writing
motorDutyCycleRolll1 = open("ev3devices/outA/duty_cycle_sp", "w")
motorDutyCyclePitch1 = open("ev3devices/outB/duty_cycle_sp", "w")
motorDutyCycleRolll2 = open("ev3devices/outC/duty_cycle_sp", "w")
motorDutyCyclePitch2 = open("ev3devices/outD/duty_cycle_sp", "w")

# Function to set the duty cycle of the motors
def SetDuty(motorDutyFileHandle, duty):
    # Clamp the value between -100 and 100
    duty = min(max(duty,-100),100)
    # Apply the signal to the motor
    FastWrite(motorDutyFileHandle, duty)
        
# Set motors in run-direct mode
with open('ev3devices/outA/command', 'w') as f:  
    f.write('run-direct')   
with open('ev3devices/outB/command', 'w') as f:
    f.write('run-direct')    
with open('ev3devices/outC/command', 'w') as f:
    f.write('run-direct')        
with open('ev3devices/outD/command', 'w') as f:
    f.write('run-direct')    

def ResetEncoders():
    with open('ev3devices/outA/position', 'w') as f:
        f.write('0')
    with open('ev3devices/outB/position', 'w') as f:
        f.write('0')       
    with open('ev3devices/outC/position', 'w') as f:
        f.write('0')       
    with open('ev3devices/outD/position', 'w') as f:
        f.write('0') 
        
########################################################################
##
## Definitions and Initialization of variables
##
########################################################################    
                
# Load tuning variables (state feedback values)          
from imp import reload
import parameters   
from parameters import *  
 
#Timing settings for the program
loopTimeMiliSec         = 20                    # Time of each loop, measured in miliseconds.
loopTimeSec             = loopTimeMiliSec/1000  # Time of each loop, measured in seconds.
motorAngleHistoryLength = 3                     # Number of previous motor angles we keep track of.

#Math constants
radiansPerDegree               = math.pi/180                                   # The number of radians in a degree.

#Platform specific constants and conversions
degPerSecondPerRawGyroUnit     = 1                                             # For the LEGO EV3 Gyro in Rate mode, 1 unit = 1 deg/s
radiansPerSecondPerRawGyroUnit = degPerSecondPerRawGyroUnit*radiansPerDegree   # Express the above as the rate in rad/s per gyro unit
degPerRawMotorUnit             = 1                                             # For the LEGO EV3 Large Motor 1 unit = 1 deg
radiansPerRawMotorUnit         = degPerRawMotorUnit*radiansPerDegree           # Express the above as the angle in rad per motor unit
RPMperPerPercentSpeed          = 1.7                                           # On the EV3, "1% speed" corresponds to 1.7 RPM (if speed control were enabled)
degPerSecPerPercentSpeed       = RPMperPerPercentSpeed*360/60                  # Convert this number to the speed in deg/s per "percent speed"
radPerSecPerPercentSpeed       = degPerSecPerPercentSpeed * radiansPerDegree   # Convert this number to the speed in rad/s per "percent speed"

# The rate at which we'll update the gyro offset (precise definition given in docs)
gyroDriftCompensationRate      = 0.1*loopTimeSec*radiansPerSecondPerRawGyroUnit

########################################################################
##
## Outer loop to start and stop the robot
##
########################################################################    

# Start up message 
print("Press Touch Sensor to Start")    
    
while True:
    # Wait for the Touch sensor to be bumped  
    WaitForTouchPress()
    WaitForTouchRelease()
       
    # Reload tuning parameters from file (which may have been updated in the meantime)
    reload(parameters)
    from parameters import *        
        
    # Reset the motors
    ResetEncoders()    
    time.sleep(0.02)   
    
    # A deque (a fifo array) which we'll use to keep track of previous motor positions, which we can use to calculate the rate of change (speed)
    motorAngleHistoryPitch = deque([0],motorAngleHistoryLength)
    motorAngleHistoryRolll = deque([0],motorAngleHistoryLength)    

    # Variables representing physical signals
    motorAngleRawPitch              = 0 # The angle of "the motor", measured in raw units (degrees for the EV3). We will take the average of both motor positions as "the motor" angle, wich is essentially how far the middle of the robot has traveled.
    motorAnglePitch                 = 0 # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
    motorAngleReferencePitch        = 0 # The reference angle of the motor. The robot will attempt to drive forward or backward, such that its measured position equals this reference (or close enough).
    motorAngleErrorPitch            = 0 # The error: the deviation of the measured motor angle from the reference. The robot attempts to make this zero, by driving toward the reference.
    motorAngleErrorAccumulatedPitch = 0 # We add up all of the motor angle error in time. If this value gets out of hand, we can use it to drive the robot back to the reference position a bit quicker.
    motorAngularSpeedPitch          = 0 # The motor speed, estimated by how far the motor has turned in a given amount of time
    motorAngularSpeedReferencePitch = 0 # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
    motorAngularSpeedErrorPitch     = 0 # The error: the deviation of the motor speed from the reference speed.
    motorDutyCyclePitch             = 0 # The 'voltage' signal we send to the motor. We calulate a new value each time, just right to keep the robot upright.
    gyroRateRawPitch                = 0 # The raw value from the gyro sensor in rate mode.
    gyroRatePitch                   = 0 # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
    gyroEstimatedAnglePitch         = 0 # The gyro doesn't measure the angle of the robot, but we can estimate this angle by keeping track of the gyroRate value in time
    gyroOffsetPitch                 = 0 # Over time, the gyro rate value can drift. This causes the sensor to think it is moving even when it is perfectly still. We keep track of this offset.

    # The same variables, but now for the roll direction
    motorAngleRawRolll              = 0 # The angle of "the motor", measured in raw units (degrees for the EV3). We will take the average of both motor positions as "the motor" angle, wich is essentially how far the middle of the robot has traveled.
    motorAngleRolll                 = 0 # The angle of the motor, converted to radians (2*pi radians equals 360 degrees).
    motorAngleReferenceRolll        = 0 # The reference angle of the motor. The robot will attempt to drive forward or backward, such that its measured position equals this reference (or close enough).
    motorAngleErrorRolll            = 0 # The error: the deviation of the measured motor angle from the reference. The robot attempts to make this zero, by driving toward the reference.
    motorAngleErrorAccumulatedRolll = 0 # We add up all of the motor angle error in time. If this value gets out of hand, we can use it to drive the robot back to the reference position a bit quicker.
    motorAngularSpeedRolll          = 0 # The motor speed, estimated by how far the motor has turned in a given amount of time
    motorAngularSpeedReferenceRolll = 0 # The reference speed during manouvers: how fast we would like to drive, measured in radians per second.
    motorAngularSpeedErrorRolll     = 0 # The error: the deviation of the motor speed from the reference speed.
    motorDutyCycleRolll             = 0 # The 'voltage' signal we send to the motor. We calulate a new value each time, just right to keep the robot upright.
    gyroRateRawRolll                = 0 # The raw value from the gyro sensor in rate mode.
    gyroRateRolll                   = 0 # The angular rate of the robot (how fast it is falling forward or backward), measured in radians per second.
    gyroEstimatedAngleRolll         = 0 # The gyro doesn't measure the angle of the robot, but we can estimate this angle by keeping track of the gyroRate value in time
    gyroOffsetRolll                 = 0 # Over time, the gyro rate value can drift. This causes the sensor to think it is moving even when it is perfectly still. We keep track of this offset.

    # Counter of the main balancing loop
    loopCount = 0      
    
    ########################################################################
    ##
    ## Calibrate Gyro
    ##
    ########################################################################    
          
     
    with open('/sys/class/power_supply/legoev3-battery/voltage_now','r') as f:
        voltage = f.read()
        print("Voltage: " + str(int(voltage)*0.000001))
          
    print("-----------------------------------")      
    print("Calibrating...")

    #As you hold the robot still, determine the average sensor value of 100 samples
    gyroRateCalibrateCount = 100
    for i in range(gyroRateCalibrateCount):
        gyroOffsetRolll = gyroOffsetRolll + FastRead(gyroRolllRaw)
        gyroOffsetPitch = gyroOffsetPitch + FastRead(gyroPitchRaw)
        time.sleep(0.01)
    gyroOffsetPitch = gyroOffsetPitch/gyroRateCalibrateCount 
    gyroOffsetRolll = gyroOffsetRolll/gyroRateCalibrateCount 
           
    # Print the result   
    print("GyroOffsetPitch: ",gyroOffsetPitch)   
    print("GyroOffsetRolll: ",gyroOffsetRolll) 
    time.sleep(0.1)


    ########################################################################
    ##
    ## MAIN BALANCING LOOP (Press Touch Sensor to stop the program)
    ##
    ########################################################################    

     
    # Time at start of loop
    tProgramStart = time.time()            
            
    while not touchSensorPressed: 

        ###############################################################
        ##  Loop info
        ###############################################################
        loopCount = loopCount + 1
        tLoopStart = time.time()
        
        ###############################################################
        ##  User input
        ###############################################################        
        
        # Driving forward and sideways
        forwardSpeedReference = 0
        leftSpeedReference    = 0
        
        # Turning the robot on the ball, with no nett balance change
        turnRate = 0

        irSensorBtn = FastRead(irSensor)
        if irSensorBtn == 1: #red up
            turnRate = 5
        elif irSensorBtn == 3: #blue up
            turnRate = -5
        elif irSensorBtn == 2: #red down
            leftSpeedReference = 8
        elif irSensorBtn == 4: #blue down
            leftSpeedReference = -8
        elif irSensorBtn == 5: #red&blue up
            forwardSpeedReference = 8
        elif irSensorBtn == 8:  # red&blue up
            forwardSpeedReference = -8

        ###############################################################
        ##  Reading the Gyro.
        ###############################################################
        gyroRateRawPitch = FastRead(gyroPitchRaw)
        gyroRateRawRolll = FastRead(gyroRolllRaw)
        gyroRatePitch = (gyroRateRawPitch - gyroOffsetPitch)*radiansPerSecondPerRawGyroUnit
        gyroRateRolll = (gyroRateRawRolll - gyroOffsetRolll)*radiansPerSecondPerRawGyroUnit

        ###############################################################
        ##  Reading the Motor Position
        ###############################################################

        motorAngleRawPitch = ((FastRead(motorEncoderPitch1) + FastRead(motorEncoderPitch2))/2)
        motorAngleRawRolll = (FastRead(motorEncoderRolll1) + FastRead(motorEncoderRolll2))/2
        motorAnglePitch = motorAngleRawPitch*radiansPerRawMotorUnit
        motorAngleRolll = motorAngleRawRolll*radiansPerRawMotorUnit

        motorAngularSpeedReferencePitch = forwardSpeedReference*radPerSecPerPercentSpeed
        motorAngularSpeedReferenceRolll =    leftSpeedReference*radPerSecPerPercentSpeed
        motorAngleReferencePitch = motorAngleReferencePitch + motorAngularSpeedReferencePitch*loopTimeSec
        motorAngleReferenceRolll = motorAngleReferenceRolll + motorAngularSpeedReferenceRolll*loopTimeSec

        motorAngleErrorPitch = motorAnglePitch - motorAngleReferencePitch  
        motorAngleErrorRolll = motorAngleRolll - motorAngleReferenceRolll   
        
        ###############################################################
        ##  Computing Motor Speed
        ###############################################################
        
        motorAngularSpeedPitch = (motorAnglePitch - motorAngleHistoryPitch[0])/(motorAngleHistoryLength*loopTimeSec)
        motorAngularSpeedRolll = (motorAngleRolll - motorAngleHistoryRolll[0])/(motorAngleHistoryLength*loopTimeSec)
        motorAngularSpeedErrorPitch = motorAngularSpeedPitch - motorAngularSpeedReferencePitch
        motorAngularSpeedErrorRolll = motorAngularSpeedRolll - motorAngularSpeedReferenceRolll
        motorAngleHistoryPitch.append(motorAnglePitch)
        motorAngleHistoryRolll.append(motorAngleRolll)
        
        
        #print motorAngularSpeedPitch, motorAngularSpeedReferencePitch, fwdSpeedControl

        ###############################################################
        ##  Computing the motor duty cycle value
        ###############################################################

        motorDutyCyclePitch =   (gainGyroAngle  * gyroEstimatedAnglePitch
                               + gainGyroRate   * gyroRatePitch
                               + gainMotorAngle * motorAngleErrorPitch
                               + gainMotorAngularSpeed * motorAngularSpeedErrorPitch
                               + gainMotorAngleErrorAccumulated * motorAngleErrorAccumulatedPitch)
                       
        motorDutyCycleRolll =   (gainGyroAngle  * gyroEstimatedAngleRolll
                               + gainGyroRate   * gyroRateRolll
                               + gainMotorAngle * motorAngleErrorRolll
                               + gainMotorAngularSpeed * motorAngularSpeedErrorRolll
                               + gainMotorAngleErrorAccumulated * motorAngleErrorAccumulatedRolll)
        
        ###############################################################
        ##  Apply the signal to the motors, and add turning
        ###############################################################

        SetDuty(motorDutyCyclePitch1, motorDutyCyclePitch+turnRate)
        SetDuty(motorDutyCyclePitch2, motorDutyCyclePitch-turnRate)
        SetDuty(motorDutyCycleRolll1, motorDutyCycleRolll+turnRate)
        SetDuty(motorDutyCycleRolll2, motorDutyCycleRolll-turnRate)

        ###############################################################
        ##  Update angle estimate and Gyro Offset Estimate
        ###############################################################

        gyroEstimatedAnglePitch = gyroEstimatedAnglePitch + gyroRatePitch*loopTimeSec
        gyroEstimatedAngleRolll = gyroEstimatedAngleRolll + gyroRateRolll*loopTimeSec
        gyroOffsetPitch = (1-gyroDriftCompensationRate)*gyroOffsetPitch+gyroDriftCompensationRate*gyroRateRawPitch
        gyroOffsetRolll = (1-gyroDriftCompensationRate)*gyroOffsetRolll+gyroDriftCompensationRate*gyroRateRawRolll

        ###############################################################
        ##  Update Accumulated Motor Error
        ###############################################################

        motorAngleErrorAccumulatedPitch = motorAngleErrorAccumulatedPitch + motorAngleErrorPitch*loopTimeSec
        motorAngleErrorAccumulatedRolll = motorAngleErrorAccumulatedRolll + motorAngleErrorRolll*loopTimeSec

        ###############################################################
        ##  Read the touch sensor (the kill switch)
        ###############################################################

        touchSensorPressed  = FastRead(touchSensorValueRaw)

        ###############################################################
        ##  Busy wait for the loop to complete
        ###############################################################
       
        while(time.time() - tLoopStart <  loopTimeSec):
            time.sleep(0.0001) 
        
    ########################################################################
    ##
    ## Closing down & Cleaning up
    ##
    ######################################################################## 

    # See if we have that world record
    tProgramEnd = time.time()    
        
    # Turn off the motors    
    SetDuty(motorDutyCyclePitch1, 0)
    SetDuty(motorDutyCyclePitch2, 0)
    SetDuty(motorDutyCycleRolll1, 0)
    SetDuty(motorDutyCycleRolll2, 0)

    # Calculate loop time
    tLoop = (tProgramEnd - tProgramStart)/loopCount
    print("Loop time:", tLoop*1000,"ms")

    # Wait for touch sensor to be released
    while touchSensorPressed: 
        touchSensorPressed = FastRead(touchSensorValueRaw) 
        time.sleep(0.1)
    tStopButtonPressDuration = time.time() - tProgramEnd

    # If a short press has occured, offer to restart program
    if(tStopButtonPressDuration < 2):
        print("Program halted. Press Touch Sensor to Restart")
    else:
        print("Exiting program")
        break
