import math
from wpilib import ADIS16470_IMU, SPI
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath import units
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from rev import CANSparkBase
from photonlibpy.photonPoseEstimator import PoseStrategy
from pathplannerlib.controller import PIDConstants as PathPlannerPIDConstants
from pathplannerlib.pathfinding import PathConstraints
from pathplannerlib.path import PathPlannerPath
from lib.classes import PIDConstants
from classes import AutoPath

class Power:
  kPowerDistributionCANId: int = 1

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: float = 0.1

class Subsystems:
  class Drive:
    kLeftRearID: int = 1
    kLeftFrontID: int = 2
    kRightRearID: int = 3
    kRightFrontID: int = 4

    kCurrentLimit: int = 60

    kDistanceFactor: float = 1.0
    kVelocityFactor: float = 1.0

    kDistanceBetweenWheels = units.inchesToMeters(6)

    kTrackWidth: float = units.inchesToMeters(24.5)
    kWheelBase: float = units.inchesToMeters(21.5)
    kDriveBaseRadius: float = Translation2d().distance(Translation2d(kWheelBase / 2, kTrackWidth / 2))

    kMaxSpeedMetersPerSecond: float = 6.32
    kMaxAngularSpeed: float = 4 * math.pi

    kDriveInputLimiter: float = 0.6
    kDriveInputRateLimit: float = 0.5

    kDriftCorrectionThetaControllerPIDConstants = PIDConstants(0.01, 0, 0, 0)
    kDriftCorrectionThetaControllerPositionTolerance: float = 0.5
    kDriftCorrectionThetaControllerVelocityTolerance: float = 0.5

    kTargetAlignmentThetaControllerPIDConstants = PIDConstants(0.075, 0, 0, 0)
    kTargetAlignmentThetaControllerPositionTolerance: float = 1.0
    kTargetAlignmentThetaControllerVelocityTolerance: float = 1.0

    kPathFollowerTranslationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFollowerRotationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFindingConstraints = PathConstraints(5.8, 3.6, units.degreesToRadians(540), units.degreesToRadians(720))

    kSwerveModuleFrontLeftDrivingMotorCANId: int = 3
    kSwerveModuleFrontLeftTurningMotorCANId: int = 4
    kSwerveModuleFrontRightDrivingMotorCANId: int = 7
    kSwerveModuleFrontRightTurningMotorCANId: int = 8
    kSwerveModuleRearLeftDrivingMotorCANId: int = 5
    kSwerveModuleRearLeftTurningMotorCANId: int = 6
    kSwerveModuleRearRightDrivingMotorCANId: int = 9
    kSwerveModuleRearRightTurningMotorCANId: int = 10

    kSwerveModuleFrontLeftOffset: float = -math.pi / 2
    kSwerveModuleFrontRightOffset: float = 0
    kSwerveModuleRearLeftOffset: float = math.pi
    kSwerveModuleRearRightOffset: float = math.pi / 2

    kSwerveModuleFrontLeftTranslation = Translation2d(kWheelBase / 2, kTrackWidth / 2)
    kSwerveModuleFrontRightTranslation =Translation2d(kWheelBase / 2, -kTrackWidth / 2)
    kSwerveModuleRearLeftTranslation = Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    kSwerveModuleRearRightTranslation = Translation2d(-kWheelBase / 2, -kTrackWidth / 2)

    kDifferentialDriveKinematics = DifferentialDriveKinematics(
      kTrackWidth
    )

    kWheelDiameterMeters: float = units.inchesToMeters(3.0)
    kDrivingMotorReduction: float = 8.46
    kDrivingEncoderPositionConversionFactor: float = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
    kDrivingEncoderVelocityConversionFactor: float = ((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) / 60.0

    class SwerveModule:
      kDrivingMotorPinionTeeth: int = 14
      kFreeSpeedRpm: float = 6238.73054766
      kWheelDiameterMeters: float = units.inchesToMeters(3.0)
      kWheelCircumferenceMeters: float = kWheelDiameterMeters * math.pi
      kDrivingMotorReduction: float = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15)
      kDrivingMotorFreeSpeedRps: float = kFreeSpeedRpm / 60
      kDriveWheelFreeSpeedRps: float = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction 
      kDrivingEncoderPositionConversionFactor: float = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction
      kDrivingEncoderVelocityConversionFactor: float = ((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) / 60.0
      kTurningEncoderInverted: bool = True
      kTurningEncoderPositionConversionFactor: float = 2 * math.pi
      kTurningEncoderVelocityConversionFactor: float = (2 * math.pi) / 60.0
      kTurningEncoderPositionPIDMinInput: float = 0
      kTurningEncoderPositionPIDMaxInput: float = kTurningEncoderPositionConversionFactor
      kDrivingMotorCurrentLimit: int = 80
      kDrivingMotorMaxReverseOutput: float = -1.0
      kDrivingMotorMaxForwardOutput: float = 1.0
      kDrivingMotorIdleMode = CANSparkBase.IdleMode.kBrake
      kDrivingMotorPIDConstants = PIDConstants(0.04, 0, 0, 1 / kDriveWheelFreeSpeedRps)
      kTurningMotorCurrentLimit: int = 20
      kTurningMotorMaxReverseOutput: float = -1.0
      kTurningMotorMaxForwardOutput: float = 1.0
      kTurningMotorIdleMode = CANSparkBase.IdleMode.kBrake
      kTurningMotorPIDConstants = PIDConstants(1, 0, 0, 0)

  class Launcher:
    # PWM ports/CAN IDs for motor controllers
    kFeederID: int = 5
    kLauncherID: int = 6

    # Current limit for launcher and feed wheels
    kLauncherCurrentLimit: int = 80
    kFeedCurrentLimit: int = 80

    # Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels in reverse
    kLauncherSpeed: float = 1.0
    kLaunchFeederSpeed: float = 1.0
    kIntakeLauncherSpeed: float = -1.0
    kIntakeFeederSpeed: float = -0.2

    kLauncherDelay: float = 1.0

class Sensors:
  class Gyro:
    kIMUAxisYaw = ADIS16470_IMU.IMUAxis.kZ
    kIMUAxisRoll = ADIS16470_IMU.IMUAxis.kY
    kIMUAxisPitch = ADIS16470_IMU.IMUAxis.kX
    kSPIPort = SPI.Port.kOnboardCS0
    kInitCalibrationTime = ADIS16470_IMU.CalibrationTime._8s
    kCommandCalibrationTime = ADIS16470_IMU.CalibrationTime._4s
    kCommandCalibrationDelay: units.seconds = 4.0

  class Pose:
    kPoseSensors: dict[str, Transform3d] = {
      # "Default": Transform3d(
      #   Translation3d(units.inchesToMeters(0), units.inchesToMeters(0), units.inchesToMeters(0)),
      #   Rotation3d(units.degreesToRadians(0), units.degreesToRadians(0), units.degreesToRadians(0))
      # )
    }
    kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kVisionSingleTagStandardDeviations: tuple[float, ...] = [1.0, 1.0, 2.0]
    kVisionMultiTagStandardDeviations: tuple[float, ...] = [0.5, 0.5, 1.0]
    kVisionMaxPoseAmbiguity: float = 0.2

_aprilTagFieldLayout = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)

class Game:
  class Field:
    kAprilTagFieldLayout = _aprilTagFieldLayout

  class Auto:
    kPaths: dict[AutoPath, PathPlannerPath] = {
      AutoPath.Test: PathPlannerPath.fromPathFile(AutoPath.Test.name)
    }
