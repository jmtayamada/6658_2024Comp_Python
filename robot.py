from wpilib import TimedRobot, Joystick, SmartDashboard
from swerveDrive import SwerveDrive
from constants import robotConstants as c
from wpimath.kinematics import ChassisSpeeds
from wpimath.controller import PIDController
from rev import CANSparkMax, SparkMaxAbsoluteEncoder, SparkMaxRelativeEncoder, CANSparkLowLevel
import numpy as np
import math

from constants import SwerveModuleConstants as s
from constants import DriveConstants as d


class Robot(TimedRobot):
    
    def robotInit(self) -> None:
        # self.drive = SwerveDrive()
        self.driveStick = Joystick(c.joystickID)
        self.testMotor = CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless)
        self.testEncoder: SparkMaxRelativeEncoder = self.testMotor.getEncoder()
        self.testEncoder.setPositionConversionFactor(s.drivingPosFactor)
        self.testEncoder.setVelocityConversionFactor(s.drivingVelFactor)
        self.PID = PIDController(s.drivingP, s.drivingI, s.drivingD)
        
        self.testVelocity = 0
        
    def teleopPeriodic(self) -> None:
        pass
        # self.drive.driveFieldRelative(ChassisSpeeds(self.driveStick.getRawAxis(0), self.driveStick.getRawAxis(1), self.driveStick.getRawAxis(2)))
        
    def testPeriodic(self) -> None:
        self.testMotor.set(self.driveStick.getRawAxis(1))
        SmartDashboard.putNumber("motor velocity", self.testEncoder.getVelocity())
        SmartDashboard.putNumber("motor position", self.testEncoder.getPosition())

        # self.testVelocity += self.PID.calculate(
        #     self.testEncoder.getVelocity(),
        #     self.driveStick.getRawAxis(1) * d.MaxSpeed
        # )
        # if self.testVelocity > 1:
        #     self.testVelocity = 1
        # if self.testVelocity < -1:
        #     self.testVelocity = -1
        # self.testMotor.set(self.testVelocity)
