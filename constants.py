from rev import CANSparkBase as CSB
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
from math import pi

class SwerveModuleConstants():
    turningP = 0.16
    turningI = 0
    turningD = 0.008
    wheelDiameter = .09
    drivingPosFactor = (.09 * pi) / 6.75  # motor to wheel conversion factor * circumference, meters
    drivingVelFactor = drivingPosFactor / 60.0  # meters per second
    turnEncoderMin = 0.0
    turnEncoderMax = 360.0
    drivingP = .04
    drivingI = 0
    drivingD = 0
    drivingMinOutput = -1.0
    drivingMaxOutput = 1.0
    drivingIdleMode = CSB.IdleMode.kBrake
    turningIdleMode = CSB.IdleMode.kBrake
    
class DriveConstants():
    FLDrivingCAN = 6
    FRDrivingCAN = 8
    RLDrivingCAN = 4
    RRDrivingCAN = 2

    FLTurningCAN = 5
    FRTurningCAN = 7
    RLTurningCAN = 3
    RRTurningCAN = 1

    FLEncoderCAN = 13
    FREncoderCAN = 12
    RLEncoderCAN = 10
    RREncoderCAN = 11

    PigeonGyro = 14
    
    halfTrackWidth = inchesToMeters(58)/2
    halfWheelBase = inchesToMeters(58)/2
    
    kinematics = SwerveDrive4Kinematics(
        Translation2d(halfWheelBase, halfTrackWidth),
        Translation2d(halfWheelBase, -halfTrackWidth),
        Translation2d(-halfWheelBase, halfTrackWidth),
        Translation2d(-halfWheelBase, -halfTrackWidth),
    )
    
    MaxSpeed = 2.0  # 3.8 meters per second
    
class IntakeConstants():
    angleID = 21
    powerID = 22
    encoderID = 23
    pitchP = 1
    pitchI = 0
    pitchD = 0
    pitchTolerance = 5
    kPitchKG = .07

class robotConstants():
    joystickID = 0
