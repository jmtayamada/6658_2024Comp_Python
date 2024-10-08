from wpilib import TimedRobot, Joystick
from swerveDrive import SwerveDrive
from constants import robotConstants as c
from wpimath.kinematics import ChassisSpeeds

from constants import DriveConstants as d
from math import pow

class Robot(TimedRobot):
    
    def robotInit(self) -> None:
        self.drive = SwerveDrive()
        self.driveStick = Joystick(c.joystickID)
        
    def teleopPeriodic(self) -> None:
        self.drive.driveFieldRelative(ChassisSpeeds(pow(self.driveStick.getRawAxis(0), 2), pow(self.driveStick.getRawAxis(1), 2), pow(self.driveStick.getRawAxis(4), 2)))
        
    def testInit(self) -> None:
        pass
        
    def testPeriodic(self) -> None:
        pass
