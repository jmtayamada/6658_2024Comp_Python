from rev import RelativeEncoder, CANSparkMax, CANSparkLowLevel
from phoenix6.hardware import CANcoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import PIDController
from constants import SwerveModuleConstants as c

class SwerveModule():
    
    def __init__(self, drivingCANId: int, turningCANId: int, encoderNum: int, reversedDrive: bool, reversedSteer: bool) -> None:
        self.drivingSparkMax = CANSparkMax(drivingCANId, CANSparkLowLevel.MotorType.kBrushless)
        self.turningSparkMax = CANSparkMax(turningCANId, CANSparkLowLevel.MotorType.kBrushless)
        self.drivingSparkMax.restoreFactoryDefaults()
        self.turningSparkMax.restoreFactoryDefaults()
        self.drivingSparkMax.setInverted(reversedDrive)
        self.turningSparkMax.setInverted(reversedSteer)
        self.drivingSparkMax.setIdleMode(c.drivingIdleMode)
        self.turningSparkMax.setIdleMode(c.turningIdleMode)
        self.drivingSparkMax.burnFlash()
        self.turningSparkMax.burnFlash()
        
        self.drivingEncoder = self.drivingSparkMax.getEncoder()
        self.drivingEncoder.setPositionConversionFactor(c.drivingPosFactor)
        self.drivingEncoder.setVelocityConversionFactor(c.drivingVelFactor)
        self.drivingEncoder.setPosition(0.0)
        self.turningEncoder = CANcoder(encoderNum)
        
        self.drivingPIDController = PIDController(c.drivingP, c.drivingI, c.drivingD)
        self.turningPIDController = PIDController(c.turningP, c.turningI, c.turningD)
        self.turningPIDController.enableContinuousInput(c.turnEncoderMin, c.turnEncoderMax)
        
        self.drivingMotorOutput = 0
        
        # self.driveReversal = reversedDrive
        
    def getCurrentRotation(self) -> Rotation2d:
        return Rotation2d.fromRotations(self.turningEncoder.get_absolute_position().value_as_double)
    
    def resetEncoders(self):
        self.drivingEncoder.setPosition(0.0)
        self.turningEncoder.set_position(0.0)
                
    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.drivingEncoder.getVelocity, self.getCurrentRotation())
        
    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drivingEncoder.getPosition(),self.getCurrentRotation())

    def setDesiredState(self, desiredState: SwerveModuleState):
        optimizedDesiredState = SwerveModuleState.optimize(desiredState, self.getCurrentRotation())
        self.turningSparkMax.set(
            -self.turningPIDController.calculate(
                self.getCurrentRotation().radians(), 
                optimizedDesiredState.angle.radians()
            )
        )
        self.drivingMotorOutput += self.drivingPIDController.calculate(
            self.drivingEncoder.getVelocity(),
            optimizedDesiredState.speed
        )
        self.drivingSparkMax.set(self.drivingMotorOutput)
