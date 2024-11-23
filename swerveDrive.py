from swerveModule import SwerveModule
from constants import DriveConstants as c
from phoenix6.hardware import Pigeon2
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d
from ntcore import NetworkTableInstance
from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder
# from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import HolonomicPathFollowerConfig, PIDConstants, ReplanningConfig
import numpy as np

class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class SwerveDrive(metaclass=Singleton):
    
    def __init__(self) -> None:
        self.moduleFL = SwerveModule(c.FLDrivingCAN, c.FLTurningCAN, c.FLEncoderCAN, True, False)
        self.moduleFR = SwerveModule(c.FRDrivingCAN, c.FRTurningCAN, c.FREncoderCAN, False, False)
        self.moduleRL = SwerveModule(c.RLDrivingCAN, c.RLTurningCAN, c.RLEncoderCAN, False, False)
        self.moduleRR = SwerveModule(c.RRDrivingCAN, c.RRTurningCAN, c.RREncoderCAN, False, False)
        
        self.swerveModuleArray = [self.moduleFL, self.moduleFR, self.moduleRL, self.moduleRR]
        
        self.gyro = Pigeon2(c.PigeonGyro)
        
        self.publishStates()
                
        self.odometry = SwerveDrive4PoseEstimator(
            c.kinematics,
            self.getHeading(),
            (
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleRL.getPosition(),
                self.moduleRR.getPosition(),
            ),
            Pose2d()
        )
        
        # # Load the RobotConfig from the GUI settings. You should probably
        # # store this in your Constants file
        # config = RobotConfig.fromGUISettings()

        # # Configure the AutoBuilder last
        # AutoBuilder.configureHolonomic(
        #     self.getPose, # Robot pose supplier
        #     self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
        #     self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        #     lambda speeds, feedforwards: self.driveRobotRelative(speeds), # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
        #     PPHolonomicDriveController( # PPHolonomicController is the built in path following controller for holonomic drive trains
        #         PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
        #         PIDConstants(5.0, 0.0, 0.0) # Rotation PID constants
        #     ),
        #     config, # The robot configuration
        #     self.shouldFlipPath, # Supplier to control path flipping based on alliance color
        #     self # Reference to this subsystem to set requirements
        # )
        
        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.getPose, # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.driveRobotRelative, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            HolonomicPathFollowerConfig( # PPHolonomicController is the built in path following controller for holonomic drive trains
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0), # Rotation PID constants
                1.0,
                1.0,
                ReplanningConfig()
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )
        
    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
        
    def publishStates(self):
        self.OdometryPublisher = NetworkTableInstance.getDefault().getStructTopic("/SwerveStates/Odometry", Pose2d).publish()
        self.RedPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates/Red", SwerveModuleState).publish()

    def updateStates(self):
        self.RedPublisher.set(self.getModuleStates())
        self.OdometryPublisher.set(self.odometry.getEstimatedPosition())
        
    def getHeading(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value_as_double)
    
    def getModuleStates(self) -> list[SwerveModuleState]:
        return [self.moduleFL.getState(), self.moduleFR.getState(), self.moduleRL.getState(), self.moduleRR.getState()]
    
    def getModulePositions(self) -> list[SwerveModulePosition]:
        return [self.moduleFL.getPosition(), self.moduleFR.getPosition(), self.moduleRL.getPosition(), self.moduleRR.getPosition()]
    
    def zeroHeading(self):
        self.gyro.set_yaw(0)
        
    def resetPose(self, pose = Pose2d()):
        self.odometry.resetPosition(self.getHeading(), self.getModulePositions(), pose)
        
    def getPose(self) -> Pose2d:
        return self.odometry.getEstimatedPosition()
    
    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        return c.kinematics.toChassisSpeeds(self.getModuleStates())
        
    def resetEncoders(self):
        self.moduleFL.resetEncoders()
        self.moduleFR.resetEncoders()
        self.moduleRL.resetEncoders()
        self.moduleRR.resetEncoders()
        
    def setModuleStates(self, desiredStates: tuple[SwerveModuleState]):
        desiredStates = c.kinematics.desaturateWheelSpeeds(desiredStates, c.MaxSpeed)
        self.moduleFL.setDesiredState(desiredStates[0])
        self.moduleFR.setDesiredState(desiredStates[1])
        self.moduleRL.setDesiredState(desiredStates[2])
        self.moduleRR.setDesiredState(desiredStates[3])
        self.odometry.update(
            self.getHeading(), 
            (
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleRL.getPosition(),
                self.moduleRR.getPosition()
            )
        )
        self.updateStates()
        self.lastDesiredSpeedFL = desiredStates[0].speed
        
    def driveFieldRelative(self, chassisSpeeds: ChassisSpeeds):
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, self.getHeading())
        moduleStates = c.kinematics.toSwerveModuleStates(speeds)
        self.setModuleStates(moduleStates)
        
    def driveRobotRelative(self, chassisSpeeds: ChassisSpeeds):
        moduleStates = c.kinematics.toSwerveModuleStates(chassisSpeeds)
        self.setModuleStates(moduleStates)
        
    def setX(self):
        self.moduleFL.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.moduleFR.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleRL.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleRR.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        
    def voltageTuning(self, voltage) -> float:
        voltages = [
            self.moduleFL.voltageControl(voltage),
            self.moduleFR.voltageControl(voltage),
            self.moduleRL.voltageControl(voltage),
            self.moduleRR.voltageControl(voltage)
        ]
        return np.average(voltages)
        