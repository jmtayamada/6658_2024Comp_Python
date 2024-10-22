from swerveModule import SwerveModule
from constants import DriveConstants as c
from phoenix6.hardware import Pigeon2
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d
from wpilib import Timer
from ntcore import NetworkTableInstance

# from pathplannerlib.auto import AutoBuilder
# from pathplannerlib.config import ReplanningConfig, HolonomicPathFollowerConfig, PIDConstants

class SwerveDrive:
    
    def __init__(self) -> None:
        self.moduleFL = SwerveModule(c.FLDrivingCAN, c.FLTurningCAN, c.FLEncoderCAN, True, False)
        self.moduleFR = SwerveModule(c.FRDrivingCAN, c.FRTurningCAN, c.FREncoderCAN, False, False)
        self.moduleRL = SwerveModule(c.RLDrivingCAN, c.RLTurningCAN, c.RLEncoderCAN, False, False)
        self.moduleRR = SwerveModule(c.RRDrivingCAN, c.RRTurningCAN, c.RREncoderCAN, False, False)
        
        self.pidTimer = Timer()

        #
        self.lastDesiredSpeedFL = 0
        self.inputArray = []
        self.outputArray = []
        self.timeArray = []
        
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
        
    def publishStates(self):
        self.OdometryPublisher = NetworkTableInstance.getDefault().getStructTopic("/SwerveStates/Odometry", Pose2d).publish()
        self.RedPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates/Red", SwerveModuleState).publish()

        self.timePublisher = NetworkTableInstance.getDefault().getFloatArrayTopic("/AutoPid/Times").publish()
        self.inputPublisher = NetworkTableInstance.getDefault().getFloatArrayTopic("/AutoPid/Input").publish()
        self.outputPublisher = NetworkTableInstance.getDefault().getFloatArrayTopic("/AutoPid/Output").publish()

    def updateStates(self):
        self.RedPublisher.set(self.getModuleStates())
        self.OdometryPublisher.set(self.odometry.getEstimatedPosition())
        
    def getHeading(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value_as_double)
    
    def getModuleStates(self):
        return [self.moduleFL.getState(), self.moduleFR.getState(), self.moduleRL.getState(), self.moduleRR.getState()]
    
    def getModulePositions(self):
        return [self.moduleFL.getPosition(), self.moduleFR.getPosition(), self.moduleRL.getPosition(), self.moduleRR.getPosition()]
    
    def zeroHeading(self):
        self.gyro.set_yaw(0)
        self.odometry.resetPosition(self.getHeading(), self.getModulePositions(), Pose2d())
        
    def resetEncoders(self):
        self.moduleFL.resetEncoders()
        self.moduleFR.resetEncoders()
        self.moduleRL.resetEncoders()
        self.moduleRR.resetEncoders()

    def startAutopidTimer(self):
        self.pidTimer.start()
        
    def setModuleStates(self, desiredStates: tuple[SwerveModuleState]):
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
        if len(self.timeArray) <= 100000:
            self.timeArray.append(self.pidTimer.get())
            self.inputArray.append(desiredStates[0].speed)
            self.outputArray.append(self.moduleFL.getState().speed)

            self.timePublisher.set(self.timeArray)
            self.inputPublisher.set(self.inputArray)
            self.outputPublisher.set(self.outputArray)


        
    def driveFieldRelative(self, chassisSpeeds: ChassisSpeeds):

        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, self.getHeading())
        moduleStates = c.kinematics.toSwerveModuleStates(speeds)
        self.setModuleStates(moduleStates)
        
    def setX(self):
        self.moduleFL.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.moduleFR.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleRL.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleRR.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        