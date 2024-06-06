import math
from wpilib import SmartDashboard
from photonlibpy.photonCamera import PhotonCamera
from lib import utils
from lib.classes import ObjectTargetInfo

class ObjectSensor:
  def __init__(
      self, 
      cameraName: str,
      objectName: str
    ) -> None:
    self._cameraName = cameraName
    self._objectName = objectName

    self._photonCamera = PhotonCamera(cameraName)

  def hasTarget(self) -> bool:
    return self._photonCamera.getLatestResult().hasTargets() if self._photonCamera.isConnected() else False

  def getTargetInfo(self) -> ObjectTargetInfo:
    if self._photonCamera.isConnected():
      result = self._photonCamera.getLatestResult()
      if result.hasTargets():
        return ObjectTargetInfo(result.getTargets()[0].getYaw(), result.getTargets()[0].getArea())
    return ObjectTargetInfo(math.nan, math.nan)
  
  def updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'Robot/Sensor/Object/{self._cameraName}/{self._objectName}/IsConnected', self._photonCamera.isConnected())
    SmartDashboard.putBoolean(f'Robot/Sensor/Object/{self._cameraName}/{self._objectName}/HasTarget', self.hasTarget())
    SmartDashboard.putString(f'Robot/Sensor/Object/{self._cameraName}/{self._objectName}/TargetInfo', utils.toJson(self.getTargetInfo()))