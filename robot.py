from wpilib import TimedRobot, Joystick, SmartDashboard
from swerveDrive import SwerveDrive
from constants import robotConstants as c
from wpimath.kinematics import ChassisSpeeds

from constants import DriveConstants as d
from math import pow

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath

from commands2.command import Command


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

        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # Another option that allows you to specify the default auto by its name
        # self.autoChooser = AutoBuilder.buildAutoChooser("My Default Auto")

        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        
    def teleopPeriodic(self) -> None:
        self.drive.driveFieldRelative(ChassisSpeeds(-self.getJoystickDeadbanded(1), -self.getJoystickDeadbanded(0), -self.getJoystickDeadbanded(4)))
        if self.driveStick.getRawButtonPressed(1):
            self.drive.zeroHeading()
        
    def testInit(self) -> None:
        pass
        
    def testPeriodic(self) -> None:
        pass
    
    def autonomousInit(self):
        auto_command = self.getAutonomousCommand()
        if auto_command == None or auto_command.isScheduled():
            return None
        auto_command.schedule()
        
    
    def autonomousPeriodic(self):
        pass
    
    def getAutonomousCommand(self):
       return self.autoChooser.getSelected()
   
    def getAutonomousPathFollow(self) -> Command:
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"))
