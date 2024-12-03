from wpilib import Joystick, SmartDashboard
from swerveDrive import SwerveDrive
from constants import robotConstants as c
from wpimath.kinematics import ChassisSpeeds

from constants import DriveConstants as d
from math import copysign

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath

from commands2.command import Command
from commands2 import TimedCommandRobot
import typing

import numpy as np
from wpilib import Timer


class Robot(TimedCommandRobot):

    autonomousCommand: typing.Optional[Command] = None

    def getJoystickDeadband(self, axis: int) -> float:
        rawAxis = self.driveStick.getRawAxis(axis)
        if(abs(rawAxis) <= d.deadband):
            return 0
        else:
            rawAxis -= d.deadband * copysign(1, rawAxis)
            rawAxis *= 1/(1-d.deadband)
            return rawAxis
    
    def robotInit(self) -> None:
        
        self.drive = SwerveDrive()
        self.driveStick = Joystick(c.joystickID)

        # Build an auto chooser. This will use Commands.none() as the default option.
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # Another option that allows you to specify the default auto by its name
        # self.autoChooser = AutoBuilder.buildAutoChooser("My Default Auto")

        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        
        self.timer = Timer()
        self.step_num = 0
        self.switch = False
        self.quasitasticForward = [[], [], []]  # time, voltage, velocity
        self.quasitasticBackward = [[], [], []]
        self.dynamicForward = [[], [], []]
        self.dynamicBackward = [[], [], []]

        self.resetRotation = False
        
    def teleopPeriodic(self) -> None:
        if self.driveStick.getRawButton(7):
            self.drive.resetRotation()
        self.drive.driveFieldRelative(ChassisSpeeds(-self.getJoystickDeadband(1), -self.getJoystickDeadband(0), -self.getJoystickDeadband(4)))
        if self.driveStick.getRawButtonPressed(1):
            self.drive.zeroHeading()
        
    def testInit(self) -> None:
        self.timer.reset()
        
    def testPeriodic(self) -> None:
        if self.step_num == 0:
            if self.switch == False:
                self.switch = True
                self.timer.start()
            if self.dynamicTesting(self.drive, self.timer.get(), True, self.dynamicForward):
                self.step_num = 1
        
        elif self.step_num == 1:
            if self.switch == True:
                self.switch = False
                self.timer.restart()
            if self.dynamicTesting(self.drive, self.timer.get(), False, self.dynamicBackward):
                self.step_num = 2
        
        elif self.step_num == 2:
            if self.switch == False:
                self.switch = True
                self.timer.restart()
            if self.quasitasticTesting(self.drive, self.timer.get(), True, self.quasitasticForward):
                self.step_num = 3
        
        elif self.step_num == 3:
            if self.switch == True:
                self.switch = False
                self.timer.restart()
            self.quasitasticTesting(self.drive, self.timer.get(), False, self.quasitasticBackward)
                
    def quasitasticTesting(self, drive: SwerveDrive, time: float, forward: bool, logList: list[list, list, list]) -> bool:
        if time < 2:
            logList[0].append(time)
            logList[1].append(2 * (forward - .5) * 12)
            logList[2].append(drive.voltageTuning(2 * (forward - .5) * 12))
            return False
        if time > 2 and time < 4:
            drive.voltageTuning(0)
            return False
        if time > 4:
            return True
    
    def dynamicTesting(self, drive: SwerveDrive, time: float, forward: bool, logList: list[list, list, list]) -> bool:
        if time < 6:
            logList[0].append(time)
            logList[1].append(2 * (forward - .5) * time)
            logList[2].append(drive.voltageTuning(2 * (forward - .5) * time))
            return False
        if time > 6 and time < 8:
            drive.voltageTuning(0)
            return False
        if time > 8:
            return True
                
    def testExit(self):
        np.savetxt("/media/sda1/quasitasticForward.txt", np.asarray(self.quasitasticForward), delimiter=", ")
        np.savetxt("/media/sda1/quasitasticBackward.txt", np.asarray(self.quasitasticBackward), delimiter=", ")
        np.savetxt("/media/sda1/dynamicForward.txt", np.asarray(self.dynamicForward), delimiter=", ")
        np.savetxt("/media/sda1/dynamicBackward.txt", np.asarray(self.dynamicBackward), delimiter=", ")
    
    def autonomousInit(self):
        self.autonomousCommand = self.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()
        
    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
    
    def autonomousPeriodic(self):
        pass
    
    def getAutonomousCommand(self):
       return self.autoChooser.getSelected()
   
    def getAutonomousPathFollow(self) -> Command:
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"))
