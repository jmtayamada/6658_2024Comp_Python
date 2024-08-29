from rev import CANSparkMax
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from phoenix6.hardware import CANcoder
from constants import IntakeConstants as c
from math import cos

class Intake():
    
    def __init__(self) -> None:
        self.intakeMotor = CANSparkMax(c.powerID)
        self.pitchMotor = CANSparkMax(c.angleID)
        self.pitchEncoder = CANcoder(c.encoderID)
        self.pitchEncoder.set_position(0)
        self.pitchController = PIDController(c.pitchP, c.pitchI, c.pitchD)
        self.pitchController.setTolerance(c.pitchTolerance)
        
    def getAngle(self) -> Rotation2d:
        return Rotation2d.fromRotations(self.pitchEncoder.get_position().value_as_double)
    
    def setAngle(self, angle: Rotation2d):
        cosineScalar = cos(self.getAngle().radians())
        self.pitchController.setSetpoint(angle.radians())
        self.pitchMotor.set(self.pitchController.calculate(self.getAngle().radians()) + cosineScalar * c.kPitchKG)
        
    def atSetPoint(self) -> bool:
        return self.pitchController.atSetpoint()
