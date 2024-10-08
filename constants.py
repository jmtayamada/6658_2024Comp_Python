from rev import CANSparkBase as CSB
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
from math import pi

class SwerveModuleConstants():
    turningP = 0.0002
    turningI = 0
    turningD = 0
    wheelDiameter = .09
    drivingPosFactor = (.09 * pi) / 6.75  # motor to wheel conversion factor * circumference, meters
    drivingVelFactor = drivingPosFactor * 60.0  # meters per second
    turnEncoderMin = 0.0
    turnEncoderMax = 360.0
    drivingP = .002
    drivingI = 0
    drivingD = 0
    drivingMinOutput = -1.0
    drivingMaxOutput = 1.0
    drivingIdleMode = CSB.IdleMode.kBrake
    turningIdleMode = CSB.IdleMode.kBrake
    
class DriveConstants():
    FLDrivingCAN = 0
    FRDrivingCAN = 1
    RLDrivingCAN = 2
    RRDrivingCAN = 3

    FLTurningCAN = 4
    FRTurningCAN = 5
    RLTurningCAN = 6
    RRTurningCAN = 7

    FLEncoderCAN = 8
    FREncoderCAN = 9
    RLEncoderCAN = 10
    RREncoderCAN = 11

    PigeonGyro = 12
    
    halfTrackWidth = inchesToMeters(28)/2
    halfWheelBase = inchesToMeters(28)/2
    
    kinematics = SwerveDrive4Kinematics(
        Translation2d(halfWheelBase, halfTrackWidth),
        Translation2d(halfWheelBase, -halfTrackWidth),
        Translation2d(-halfWheelBase, halfTrackWidth),
        Translation2d(-halfWheelBase, -halfTrackWidth),
    )
    
    MaxSpeed = 5.0  # meters per second
    
class IntakeConstants():
    angleID = 4
    powerID = 5
    encoderID = 6
    pitchP = 1
    pitchI = 0
    pitchD = 0
    pitchTolerance = 5
    kPitchKG = .07

class robotConstants():
    joystickID = 0
