import sys
import time

import rtde_receive
import rtde_control

from threading import Thread
from math import pi
from Flexible_Clinet import FlexibleClient #241231_plotjuggler

class UR_State(Thread):
    plotjuggler = False
    refreshRate = 0.01 # Second
    connection = False
    stop_state = {'ProtectiveStopped': False, 'EmergencyStopped': False}
    robot_state = {'RobotMode': '', 'RobotStatus': '', 'SafetyMode': '', 'ActualRobotVoltage': 0, 'ActualRobotCurrent': 0}
    joint_state = {'actualQ': [0,0,0,0,0,0], 'actualQd': [0,0,0,0,0,0], 'actualCurrent': [0,0,0,0,0,0]}
    tcp_state = {'actualPos': [0,0,0,0,0,0], 'actualSpeed': [0,0,0,0,0,0], 'actualForce': [0,0,0,0,0,0]}
    
    def __init__(self, IP_ADDRESS='192.168.0.44', plotjuggler = True):
        try:
            super().__init__()
            self.rtde_r = rtde_receive.RTDEReceiveInterface(IP_ADDRESS)
            self.rtde_c = rtde_control.RTDEControlInterface(IP_ADDRESS)
            self.plotjuggler = plotjuggler
            if self.plotjuggler:  #241231_plotjuggler
                try:
                    self.UDP_client = FlexibleClient(protocol='UDP')
                    print(">> PlotJuggler successfully connected.")
                except Exception as e:
                    print(e)
        except Exception as e:
            self.connection = False
            print(e)
            print('<<ERROR>>: Try again')
        else:
            self.connection = True
            print('>> UR Successfully Connected.', )
        
    def run(self):
        try:
            # print("Start local thread.")
            while True:
                self.stop_state['ProtectiveStopped'] = self.isProtectiveStopped()
                self.stop_state['EmergencyStopped'] = self.isEmergencyStopped()
                self.robot_state['ActualRobotVoltage'] = self.getActualRobotVoltage()
                self.robot_state['ActualRobotCurrent'] = self.getActualRobotCurrent()
                self.robot_state['RobotMode'] = self.getRobotMode()
                self.robot_state['RobotStatus'] = self.getRobotStatus()
                self.robot_state['SafetyMode'] = self.getSafetyMode()
                self.joint_state['actualQ'] = self.getActualQ()
                self.joint_state['actualQd'] = self.getActualQd()
                self.joint_state['actualCurrent'] = self.getActualCurrent()
                self.tcp_state['actualPos'] = self.getActualTCPPose()
                self.tcp_state['actualSpeed'] = self.getActualTCPSpeed()
                # self.tcp_state['actualForce'] = self.getFtRawWrench()
                self.tcp_state['actualForce'] = self.getActualTCPForce() 
                if self.plotjuggler:  #241231_plotjuggler             
                    self.UDP_client.send_data(self.joint_state)
                    self.UDP_client.send_data(self.tcp_state) 
                    # print(self.joint_state)                
                time.sleep(self.refreshRate)

        except KeyboardInterrupt:
            self.disconnect()
            print("\nDisconnect the communication.")
        except Exception as e:
            self.connection = False
            self.disconnect()
            print(e)
    # Read UR Status
    def readUR(self):
        try:
            self.stop_state['ProtectiveStopped'] = self.isProtectiveStopped()
            self.stop_state['EmergencyStopped'] = self.isEmergencyStopped()
            self.robot_state['ActualRobotVoltage'] = self.getActualRobotVoltage()
            self.robot_state['ActualRobotCurrent'] = self.getActualRobotCurrent()
            self.robot_state['RobotMode'] = self.getRobotMode()
            self.robot_state['RobotStatus'] = self.getRobotStatus()
            self.robot_state['SafetyMode'] = self.getSafetyMode()
            self.joint_state['actualQ'] = self.getActualQ()
            self.joint_state['actualQd'] = self.getActualQd()
            self.joint_state['actualCurrent'] = self.getActualCurrent()
            self.tcp_state['actualPos'] = self.getActualTCPPose()
            self.tcp_state['actualSpeed'] = self.getActualTCPSpeed()
            # self.tcp_state['actualForce'] = self.getFtRawWrench()
            self.tcp_state['actualForce'] = self.getActualTCPForce()
            # if self.plotjuggler:  #241231_plotjuggler             
            #      self.UDP_client.send_data(self.joint_state)           
        except Exception as e:
            self.connection = False
            self.disconnect()
            print(e)
    
    # Can be used to disconnect from the robot
    def disconnect(self):
        self.connection = False
        try:
            self.rtde_r.disconnect()
            self.rtde_c.disconnect()
            print(">> UR Disconnected.")
        except Exception as e:
            print(e)
                
            
  
    # Can be used to reconnect to the robot after a lost connection.
    def reconnect(self):
        return self.rtde_r.reconnect() # bool
    
    # Connection status for RTDE, useful for checking for lost connection.
    def isConnected(self):
        return self.rtde_r.isConnected() # bool
    
    # Used for waiting the rest of the control period, set implicitly as dt = 1 / frequency.
    def waitPeriod(self):
        self.rtde_r.waitPeriod()
        
    # This function is used in combination with waitPeriod() and is used to get the start of a control period / cycle.
    def initPeriod(self):
        return self.rtde_r.initPeriod()
    
    # Time elapsed since the controller was started [s]
    def getTimestamp(self):
        return self.rtde_r.getTimestamp() # double
    
    # Target joint positions
    def getTargetQ(self):
        return self.rtde_r.getTargetQ() # vector<double>
    
    # Target joint velocities
    def getTargetQd(self):
        return self.rtde_r.getTargetQd() # vector<double>
    
    # Target joint currents
    def getTargetCurrent(self):
        return self.rtde_r.getTargetCurrent() # vector<double>
    
    # Target joint moments(torques)
    def getTargetMoment(self):
        return self.rtde_r.getTargetMoment() # vector<double>
          
    # Actual joint positions
    def getActualQ(self):
        return self.rtde_r.getActualQ()
    
    # Actual joint velocities
    def getActualQd(self):
        return self.rtde_r.getActualQd()
    
    # Actual joint currents
    def getActualCurrent(self):
        return self.rtde_r.getActualCurrent()
    
    # Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
    def getActualTCPPose(self):
        return self.rtde_r.getActualTCPPose()
    
    # Actual speed of the tool given in Cartesian coordinates
    def getActualTCPSpeed(self):
        return self.rtde_r.getActualTCPSpeed()
        
    # Generalized forces in the TCP
    def getFtRawWrench(self):
        return self.rtde_r.getFtRawWrench()
    
        # Generalized forces in the TCP
    def getActualTCPForce(self):
        return self.rtde_r.getActualTCPForce()
    
    # Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
    def getTargetTCPPose(self):
        return self.rtde_r.getTargetTCPPose()
    
    # Target speed of the tool given in Cartesian coordinates
    def getTargetTCPSpeed(self):
        return self.rtde_r.getTargetTCPSpeed()
    
    # Temperature of each joint in degrees Celsius
    def getJointTemperatures(self):
        return self.rtde_r.getJointTemperatures()
    
    # Controller real-time thread execution time
    def getActualExecutionTime(self):
        return self.rtde_r.getActualExecutionTime() # double
    
    # Robot mode
    # -1 = ROBOT_MODE_NO_CONTROLLER 0 = ROBOT_MODE_DISCONNECTED 1 = ROBOT_MODE_CONFIRM_SAFETY 2 = ROBOT_MODE_BOOTING 3 = ROBOT_MODE_POWER_OFF 
    # 4 = ROBOT_MODE_POWER_ON 5 = ROBOT_MODE_IDLE 6 = ROBOT_MODE_BACKDRIVE 7 = ROBOT_MODE_RUNNING 8 = ROBOT_MODE_UPDATING_FIRMWARE
    def getRobotMode(self):
        mode = self.rtde_r.getRobotMode() # int32
        if mode == -1: return 'No Ctrl'
        elif mode == 0: return 'Disconnect'
        elif mode == 1: return 'Confirm_Safety'
        elif mode == 2: return 'Booting'
        elif mode == 3: return 'Power Off'
        elif mode == 4: return 'Power On'
        elif mode == 5: return 'Idle'
        elif mode == 6: return 'Backdirve'
        elif mode == 7: return 'Running'
        elif mode == 8: return 'Update Firm'
        else: return 'None'
        
    # Robot status Bits 0-3: Is power on | Is program running | Is teach button pressed | Is power button pressed
    def getRobotStatus(self):
        status = self.rtde_r.getRobotStatus() # uint32
        if status == 0: return 'Power off'
        elif status == 1: return 'Power on'
        elif status == 2: return 'Running'
        elif status == 4: return 'Teaching'
        elif status == 8: return 'Pwr Btn'
        else: return 'None'
    
    # Joint control modes
    def getJointMode(self):
        return self.rtde_r.getJointMode() # vector<int32>
    
    # Safefy mode
    def getSafetyMode(self):
        safetyMode = self.rtde_r.getSafetyMode() # int32
        if safetyMode == 0: return 'Normal'
        elif safetyMode == 1: return 'Reduced'
        elif safetyMode == 2: return 'Protective stopped'
        elif safetyMode == 3: return 'Recovery'
        elif safetyMode == 4: return 'Safeguard stop'
        elif safetyMode == 5: return 'System emergency'
        elif safetyMode == 6: return 'Robot emergency'
        elif safetyMode == 7: return 'Emegency stop'
        elif safetyMode == 8: return 'Violation'
        elif safetyMode == 7: return 'Fault'
        elif safetyMode == 8: return 'Stop due safety'
        else: return 'None'    
        
    # Safety status bits Bits 0-10: Is normal mode | Is reduced mode | Is protective stopped | Is recovery mode
    #                       | Is safeguard stopped | Is system emergency stopped | Is robot emergency stopped
    #                       |Is emergency stopped | Is violation | Is fault | Is stopped due to safety
    def getSafetyStatusBits(self):
        return self.rtde_r.getSafetyStatusBits() # int32
    
    # Safety Control Board: Main voltage
    def getActualMainVoltage(self):
        return self.rtde_r.getActualMainVoltage() # double
    
    # Safety Control Board: Robot voltage (48V)
    def getActualRobotVoltage(self):
        return self.rtde_r.getActualRobotVoltage() # double
    
    # Safety Control Board: Robot current
    def getActualRobotCurrent(self):
        return self.rtde_r.getActualRobotCurrent() # double
    
    # Actual joint voltages
    def getActualJointVoltage(self):
        return self.rtde_r.getActualJointVoltage() # vector<double>
    
    # Program state
    def getRuntimeState(self):
        return self.rtde_r.getRuntimeState() # uint32
    
    # a bool indicating if the robot is in ‘Protective stop’
    def isProtectiveStopped(self):
        return self.rtde_r.isProtectiveStopped() # bool
    
    # a bool indicating if the robot is in ‘Emergency stop’
    def isEmergencyStopped(self):
        return self.rtde_r.isEmergencyStopped() # bool
    
    # Get the payload of the robot in [kg].
    def getPayload(self):
        return self.rtde_r.getPayload() # double
    
    # Get the payload Center of Gravity (CoGx, CoGy, CoGz) in [m]
    def getPayloadCog(self):
        return self.rtde_r.getPayloadCog() # vector<double>
    
    '''
    RTDE Control Function
    '''
    def RelativeMove(self):
        target = self.tcp_state['actualPos']
        print(target)
        target[2] += 0.1
        
        self.rtde_c.moveL(target, 0.25, 0.5, True)
        time.sleep(0.1)
        self.rtde_c.stopL(0.5)
        # self.rtde_c.stopScript()
        
    def stopL(self, a = 0.5):
        self.rtde_c.stopL(a)
        
    def stopJ(self):
        self.rtde_c.stopJ()
        
    # bool moveJ(const std::vector<double> &q, double speed = 1.05, double acceleration = 1.4, bool asynchronous = false)
    # a bool specifying if the move command should be asynchronous. If asynchronous is true it is possible to stop a move command using either the stopJ or stopL function.
    # Default is false, this means the function will block until the movement has completed.
    def moveJ(self, target_q, speed = 1.05, acceleration = 1.4, asynchronous = False):
        return self.rtde_c.moveJ(target_q, speed, acceleration, asynchronous)
        
    def moveJ_IK(self, target_q, speed = 1.05, acceleration = 1.4, asynchronous = False):
        return self.rtde_c.moveJ_IK(target_q, speed, acceleration, asynchronous)
        
    # bool moveL(const std::vector<double> &pose, double speed = 0.25, double acceleration = 1.2, bool asynchronous = false)
    # a bool specifying if the move command should be asynchronous. If asynchronous is true it is possible to stop a move command using either the stopJ or stopL function.
    # Default is false, this means the function will block until the movement has completed.
    def moveL(self, target_pos, speed = 0.25, acceleration = 1.2, asynchronous = False):
        return self.rtde_c.moveL(target_pos, speed, acceleration, asynchronous)
    
    def moveL_FK(self, target_pos, speed = 0.25, acceleration = 1.2, asynchronous = False):
        return self.rtde_c.moveL_FK(target_pos, speed, acceleration, asynchronous)
    
    def getForwardKinematics(self):
        return self.rtde_c.getForwardKinematics()
    
    def getInverseKinematics(self, target_pos):
        return self.rtde_c.getInverseKinematics(target_pos)
    
    def speedJ(self, target_qd, acceleration = 0.5, time = 0.0):
        return self.rtde_c.speedJ(target_qd, acceleration, time)
    
    def speedL(self, target_xd, acceleration = 0.25, time = 0.0):
        return self.rtde_c.speedL(target_xd, acceleration, time)
    
    def speedStop(self, a = 10.0):
        return self.rtde_c.speedStop(a)
    
    def isSteady(self):
        retVal = self.rtde_c.isSteady()
        return retVal
        
    
    # Mass in kilograms
    # Center of Gravity, a vector [CoGx, CoGy, CoGz] specifying the displacement (in meters) from the toolmount.
    # If not specified the current CoG will be used.
    def setPayLoad(self, mass, COG):
        return self.rtde_c.setPayLaod(mass, COG)
        
    def setTcp(self, tcp_offset):
        return self.rtde_c.setTcp(tcp_offset)
    
    def getTCPOffset(self):
        return self.rtde_c.getTCPOffset()
        
    def teachMode(self):
        return self.rtde_c.teachMode()
    
    def endTeachMode(self):
        return self.rtde_c.endTeachMode()
    
    def moveUntilContact(self, xd, direction = {0, 0, 0, 0, 0, 0}, acceleration = 0.5):
        return self.rtde_c.moveUntilContact(xd, direction, acceleration)
    
    # Robot status Bits 0-3: Is power on | Is program running | Is teach button pressed | Is power button pressed
    # There is a synchronization gap between the three interfaces RTDE Control RTDE Receive and Dashboard Client.
    # RTDE Control and RTDE Receive open its own RTDE connection and so the internal state is not in sync.
    # That means, if RTDE Control reports, that program is running, RTDE Receive may still return that program is not running.
    # The update of the Dashboard Client even needs more time.
    # That means, the dashboard client still returns program not running after some milliseconds have passed after RTDE Control already reports program running.
    # def getRobotStatus(self):
    #     return self.rtde_c.getRobotStatus()
    
    def getActualToolFlangePos(self):
        return self.rtde_c.getActualToolFlangePos()
    
    def zeroFtSensor(self):
        return self.rtde_c.zeroFtSensor()
        
'''
Local function declaration
'''
def rad2deg(value):
    if type(value) is list:
        temp = []
        for i in range(6):
            temp.append(value[i]*(180.0/pi))
        return temp
    else:
        return value*(180.0/pi)
        

    
    
if __name__ == '__main__':
    recv_instance = UR_State()
    pass