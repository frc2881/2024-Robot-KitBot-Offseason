from typing import Callable
import math
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, SendableChooser
from wpilib.drive import DifferentialDrive
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveKinematics, DifferentialDriveWheelSpeeds
from rev import CANSparkBase, CANSparkMax, CANSparkLowLevel
from lib import utils, logger
from lib.classes import SwerveModuleLocation, DriveSpeedMode, DriveOrientation, DriveDriftCorrection, DriveLockState
from lib.subsystems.drive.swerve_module import SwerveModule
import constants

class DriveSubsystem(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], float]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Drive

    self._getGyroHeading: Callable[[], float] = getGyroHeading

    self._leftFront = CANSparkMax(self._constants.kLeftFrontID, CANSparkLowLevel.MotorType.kBrushless)
    self._leftRear = CANSparkMax(self._constants.kLeftRearID, CANSparkLowLevel.MotorType.kBrushless)
    self._rightFront = CANSparkMax(self._constants.kRightFrontID, CANSparkLowLevel.MotorType.kBrushless)
    self._rightRear = CANSparkMax(self._constants.kRightRearID, CANSparkLowLevel.MotorType.kBrushless)

    self._leftFront.setSmartCurrentLimit(self._constants.kCurrentLimit)
    self._leftRear.setSmartCurrentLimit(self._constants.kCurrentLimit)
    self._rightFront.setSmartCurrentLimit(self._constants.kCurrentLimit)
    self._rightRear.setSmartCurrentLimit(self._constants.kCurrentLimit)

    self._leftRear.follow(self._leftFront)
    self._rightRear.follow(self._rightFront)

    self._leftFront.setInverted(False)
    self._rightFront.setInverted(True)

    self._drivetrain = DifferentialDrive(self._leftFront, self._rightFront)
    self._leftEncoder = self._leftFront.getEncoder()
    self._leftEncoder.setPositionConversionFactor(self._constants.kDrivingEncoderPositionConversionFactor)
    self._leftEncoder.setVelocityConversionFactor(self._constants.kDrivingEncoderVelocityConversionFactor)
    self._rightEncoder = self._rightFront.getEncoder()
    self._rightEncoder.setPositionConversionFactor(self._constants.kDrivingEncoderPositionConversionFactor)
    self._rightEncoder.setVelocityConversionFactor(self._constants.kDrivingEncoderVelocityConversionFactor)
    self._kinematics = DifferentialDriveKinematics(self._constants.kDistanceBetweenWheels) 
    # setIdleMode
    # burnFlash

    self._isAlignedToTarget: bool = False
    self._targetAlignmentThetaController = PIDController(
      self._constants.kTargetAlignmentThetaControllerPIDConstants.P, 
      self._constants.kTargetAlignmentThetaControllerPIDConstants.I, 
      self._constants.kTargetAlignmentThetaControllerPIDConstants.D
    )
    self._targetAlignmentThetaController.enableContinuousInput(-180.0, 180.0)
    self._targetAlignmentThetaController.setTolerance(
      self._constants.kTargetAlignmentThetaControllerPositionTolerance, 
      self._constants.kTargetAlignmentThetaControllerVelocityTolerance
    )

  def resetEncoder(self) -> None:
    self._leftEncoder.setPosition(0) 
    self._rightEncoder.setPosition(0)

  def getDistance(self) -> float:
    return self._leftEncoder.getPosition()
  
  def periodic(self) -> None:
    self._updateTelemetry()

  def getSpeeds(self) -> ChassisSpeeds:
    wheelSpeeds = DifferentialDriveWheelSpeeds(
      self._leftEncoder.getVelocity(),
      self._rightEncoder.getVelocity()
    )
    return self._constants.kDifferentialDriveKinematics.toChassisSpeeds(wheelSpeeds)
  
  def driveWithSpeeds(self, speeds: ChassisSpeeds) -> None:
    wheelSpeeds = self._constants.kDifferentialDriveKinematics.toWheelSpeeds(speeds)
    self._drivetrain.tankDrive(wheelSpeeds.left, wheelSpeeds.right)

  def driveWithControllerCommand(
      self, 
      getLeftY: Callable[[], float], 
      getRightX: Callable[[], float]
    ) -> Command:
    return self.run(
      lambda: self._arcadeDrive(getLeftY(), getRightX())
    ).withName("DriveWithController")
  
  def getLeftEncoderPosition(self) -> float:
    return self._leftEncoder.getPosition()
  
  def getRightEncoderPosition(self) -> float:
    return self._rightEncoder.getPosition()
  
  def _arcadeDrive(self, speed: float, rotation: float) -> None:
    self._drivetrain.arcadeDrive(speed, rotation, True)

  def alignToTargetCommand(self, getRobotPose: Callable[[], Pose2d], getTargetYaw: Callable[[], float]) -> Command:
    return self.run(
      lambda: self._alignToTarget(getRobotPose().rotation().degrees(), getTargetYaw())
    ).beforeStarting(
      lambda: [
        self.clearTargetAlignment(),
        self._targetAlignmentThetaController.setSetpoint(getTargetYaw()),
        self._targetAlignmentThetaController.reset()
      ]
    ).until(
      lambda: self._isAlignedToTarget
    )

  def _alignToTarget(self, robotYaw: float, targetYaw: float) -> None:
    speedRotation: float = self._targetAlignmentThetaController.calculate(robotYaw)
    speedRotation += math.copysign(0.15, speedRotation)
    if self._targetAlignmentThetaController.atSetpoint():
      speedRotation = 0
      self._isAlignedToTarget = True
    self._arcadeDrive(0, 0)

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def reset(self) -> None:
    self._arcadeDrive(0, 0)
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTarget", self._isAlignedToTarget)
  