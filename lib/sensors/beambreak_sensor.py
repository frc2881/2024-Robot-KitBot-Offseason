from wpilib import DigitalInput, SmartDashboard

class BeamBreakSensor:
  def __init__(
      self, 
      sensorName: str,
      channel: int
    ) -> None:
    self._sensorName = sensorName
    self._digitalInput = DigitalInput(channel)
    
    self.isTriggered: bool = False

  def hasTarget(self) -> bool:
    hasTarget = not self._digitalInput.get()
    if hasTarget and not self.isTriggered:
      self.isTriggered = True
    return hasTarget
  
  def resetTrigger(self) -> None:
    self.isTriggered = False

  def updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'Robot/Sensor/BeamBreak/{self._sensorName}/HasTarget', self.hasTarget())
    SmartDashboard.putBoolean(f'Robot/Sensor/BeamBreak/{self._sensorName}/IsTriggered', self.isTriggered)