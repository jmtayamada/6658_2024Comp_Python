from wpilib import TimedRobot, Joystick
from swerveDrive import SwerveDrive
from constants import robotConstants as c
from wpimath.kinematics import ChassisSpeeds


class Robot(TimedRobot):
    
    def robotInit(self) -> None:
        self.drive = SwerveDrive()
        self.driveStick = Joystick(c.joystickID)
        
    def teleopPeriodic(self) -> None:
        self.drive.driveFieldRelative(ChassisSpeeds(self.driveStick.getRawAxis(0), self.driveStick.getRawAxis(1), self.driveStick.getRawAxis(2)))