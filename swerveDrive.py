from swerveModule import SwerveModule
from constants import DriveConstants as c
from phoenix6.hardware import Pigeon2
from wpimath.kinematics import SwerveDrive4Odometry, ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Rotation2d, Pose2d

# from pathplannerlib.auto import AutoBuilder
# from pathplannerlib.config import ReplanningConfig, HolonomicPathFollowerConfig, PIDConstants

class SwerveDrive():
    
    def __init__(self) -> None:
        self.moduleFL = SwerveModule(c.FLDrivingCAN, c.FLTurningCAN, c.FLEncoderCAN, False, False)
        self.moduleFR = SwerveModule(c.FRDrivingCAN, c.FRTurningCAN, c.FREncoderCAN, False, False)
        self.moduleRL = SwerveModule(c.RLDrivingCAN, c.RLTurningCAN, c.RLEncoderCAN, False, False)
        self.moduleRR = SwerveModule(c.RRDrivingCAN, c.RRTurningCAN, c.RREncoderCAN, False, False)
                
        self.gyro = Pigeon2(c.PigeonGyro)
                
        self.odometry = SwerveDrive4Odometry(
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
        
    def getHeading(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro.get_yaw().value_as_double)
    
    def zeroHeading(self):
        self.gyro.set_yaw(0)
        
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
        
    def driveFieldRelative(self, chassisSpeeds: ChassisSpeeds):
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, self.getHeading())
        moduleStates = c.kinematics.toSwerveModuleStates(speeds)
        self.setModuleStates(moduleStates)
        
    def setX(self):
        self.moduleFL.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.moduleFR.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleRL.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleRR.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        