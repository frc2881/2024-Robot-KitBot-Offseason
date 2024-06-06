from wpilib import SmartDashboard
from lib import utils

class DistanceSensor:
  def __init__(
      self, 
      sensorName: str,
      minTargetDistance: float,
      maxTargetDistance: float
    ) -> None:
    self._sensorName = sensorName
    self._minTargetDistance = minTargetDistance
    self._maxTargetDistance = maxTargetDistance
    
    self.isTriggered: bool = False

  def getDistance(self) -> float:
    return SmartDashboard.getEntry(f'Robot/Sensor/Distance/{self._sensorName}').getFloat()

  def hasTarget(self) -> bool:
    hasTarget = utils.isValueInRange(self.getDistance(), self._minTargetDistance, self._maxTargetDistance)
    if hasTarget and not self.isTriggered:
      self.isTriggered = True
    return hasTarget
  
  def resetTrigger(self) -> None:
    self.isTriggered = False

  def updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'Robot/Sensor/Distance/{self._sensorName}/HasTarget', self.hasTarget())
    SmartDashboard.putBoolean(f'Robot/Sensor/Distance/{self._sensorName}/IsTriggered', self.isTriggered)