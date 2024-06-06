from wpilib import SmartDashboard
from wpimath.geometry import Transform3d
from robotpy_apriltag import AprilTagFieldLayout
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy, EstimatedRobotPose

class PoseSensor:
  def __init__(
      self, 
      cameraName: str,
      cameraTransform: Transform3d,
      poseStrategy: PoseStrategy,
      fallbackPoseStrategy: PoseStrategy,
      aprilTagFieldLayout: AprilTagFieldLayout
    ) -> None:
    self._cameraName = cameraName
    
    self._photonCamera = PhotonCamera(cameraName)
    self._photonCamera.setDriverMode(False)
    self._photonPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, self._photonCamera, cameraTransform)
    self._photonPoseEstimator.multiTagFallbackStrategy = fallbackPoseStrategy

  def getEstimatedRobotPose(self) -> EstimatedRobotPose | None:
    if self._photonCamera.isConnected():
        return self._photonPoseEstimator.update()
    return None
  
  def hasTarget(self) -> bool:
    if self._photonCamera.isConnected():
      return self._photonCamera.getLatestResult().hasTargets()
    return False
  
  def updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'Robot/Sensor/Pose/{self._cameraName}/IsConnected', self._photonCamera.isConnected())
    SmartDashboard.putBoolean(f'Robot/Sensor/Pose/{self._cameraName}/HasTarget', self.hasTarget())