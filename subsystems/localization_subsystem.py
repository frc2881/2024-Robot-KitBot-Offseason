from typing import Callable
from commands2 import Subsystem
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d, Pose3d, Transform2d, Transform3d, Rotation3d
from wpimath.estimator import DifferentialDrivePoseEstimator
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib.sensors.pose_sensor import PoseSensor
from lib import utils, logger
import constants

class LocalizationSubsystem(Subsystem):
  def __init__(
      self,
      poseSensors: list[PoseSensor],
      getGyroRotation: Callable[[], Rotation2d],
      getLeftEncoderPosition: Callable[[], float],
      getRightEncoderPosition: Callable[[], float]
    ) -> None:
    super().__init__()

    self._poseSensors = poseSensors
    self._getGyroRotation = getGyroRotation
    self._getLeftEncoderPosition = getLeftEncoderPosition
    self._getRightEncoderPosition = getRightEncoderPosition

    self._poseEstimator = DifferentialDrivePoseEstimator(
      constants.Subsystems.Drive.kDifferentialDriveKinematics,
      self._getGyroRotation(),
      self._getLeftEncoderPosition(),
      self._getRightEncoderPosition(),
      Pose2d()
    )

  def periodic(self) -> None:
    self._updatePose()
    self._updateTelemetry()

  def getPose(self) -> Pose2d:
    return self._poseEstimator.getEstimatedPosition()

  def _updatePose(self) -> None:
    self._poseEstimator.update(self._getGyroRotation(), self._getLeftEncoderPosition(), self._getRightEncoderPosition())
    for poseSensor in self._poseSensors:
      estimatedRobotPose = poseSensor.getEstimatedRobotPose()
      if estimatedRobotPose is not None:
        pose = estimatedRobotPose.estimatedPose.toPose2d()
        if self._isPoseOnField(pose):
          if estimatedRobotPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR:
            self._poseEstimator.addVisionMeasurement(
              pose,
              estimatedRobotPose.timestampSeconds,
              constants.Sensors.Pose.kVisionMultiTagStandardDeviations
            )
          else:
            for target in estimatedRobotPose.targetsUsed:
              if utils.isValueInRange(target.getPoseAmbiguity(), 0, constants.Sensors.Pose.kVisionMaxPoseAmbiguity):
                self._poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds, constants.Sensors.Pose.kVisionSingleTagStandardDeviations)
                break

  def resetPose(self, pose: Pose2d) -> None:
    # NO-OP as current pose is always maintained by pose sensors in the configuration for this robot
    # self._poseEstimator.resetPosition(self._getGyroRotation(), self._getSwerveModulePositions(), pose)
    pass   

  def _isPoseOnField(self, pose: Pose2d) -> bool:
    x: float = pose.X()
    y: float = pose.Y()
    return (
      (x >= 0 and x <= constants.Game.Field.kAprilTagFieldLayout.getFieldLength()) 
      and 
      (y >= 0 and y <= constants.Game.Field.kAprilTagFieldLayout.getFieldWidth())
    )

  def hasVisionTargets(self) -> bool:
    for poseSensor in self._poseSensors:
      if poseSensor.hasTarget():
        return True
    return False

  def getTargetPose(self) -> Pose3d:
    return utils.getValueForAlliance(
      Pose3d(), 
      Pose3d()
    )

  def getTargetYaw(self) -> float:
    robotPose = self.getPose()
    targetPose = self.getTargetPose().toPose2d()
    targetTranslation = targetPose.relativeTo(robotPose).translation()
    targetRotation = Rotation2d(targetTranslation.X(), targetTranslation.Y()).rotateBy(Rotation2d.fromDegrees(0)).rotateBy(robotPose.rotation())
    return utils.wrapAngle(targetRotation.degrees())
  
  def getTargetPitch(self) -> float:
    return utils.getPitchToPose(
      Pose3d(self.getPose()),
      self.getTargetPose()
    )
  
  def getTargetDistance(self) -> float:
    return utils.getDistanceToPose(
      self.getPose(),
      self.getTargetPose().toPose2d()
    )

  def _updateTelemetry(self) -> None:
    robotPose = self.getPose()
    SmartDashboard.putNumberArray("Robot/Pose/Current", [robotPose.X(), robotPose.Y(), robotPose.rotation().degrees()])
    SmartDashboard.putNumber("Robot/Pose/Target/Yaw", self.getTargetYaw())
    SmartDashboard.putNumber("Robot/Pose/Target/Pitch", self.getTargetPitch())
    SmartDashboard.putNumber("Robot/Pose/Target/Distance", self.getTargetDistance())