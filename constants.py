from rev import CANSparkBase as CSB
from wpimath.units import inchesToMeters
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d

class SwerveModuleConstants():
    turningP = 1
    turningI = 0
    turningD = 0
    drivingPosFactor = 1.0
    drivingVelFactor = drivingPosFactor * 60.0
    turnEncoderMin = 0.0
    turnEncoderMax = 360.0
    drivingP = 1
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
    
    MaxSpeed = 5.0
    
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
