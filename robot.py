from wpilib import TimedRobot, Joystick
from swerveDrive import SwerveDrive
from constants import robotConstants as c
from wpimath.kinematics import ChassisSpeeds

from constants import DriveConstants as d
from math import pow

class Robot(TimedRobot):


    def getJoystickDeadbanded(self, axis: int) -> float:
        rawAxis = self.driveStick.getRawAxis(axis)
        if(abs(rawAxis) <= d.deadband):
            return 0
        else:
            return rawAxis
    
    def robotInit(self) -> None:
        self.drive = SwerveDrive()
        self.driveStick = Joystick(c.joystickID)
        
    def teleopPeriodic(self) -> None:
        self.drive.driveFieldRelative(ChassisSpeeds(-self.getJoystickDeadbanded(1), -self.getJoystickDeadbanded(0), -self.getJoystickDeadbanded(4)))
        if self.driveStick.getRawButtonPressed(1):
            self.drive.zeroHeading()
        
    def testInit(self) -> None:
        pass
        
    def testPeriodic(self) -> None:
        pass
