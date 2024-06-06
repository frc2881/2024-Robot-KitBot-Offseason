from commands2 import Command, cmd
from wpilib import ADIS16470_IMU, SPI, SmartDashboard, RobotBase
from wpimath.geometry import Rotation2d, Pose2d
from lib import utils

class GyroSensor(ADIS16470_IMU):
  def __init__(
      self, 
      yawAxis: ADIS16470_IMU.IMUAxis, 
      pitchAxis: ADIS16470_IMU.IMUAxis, 
      rollAxis: ADIS16470_IMU.IMUAxis, 
      port: SPI.Port, 
      initCalibrationTime: ADIS16470_IMU.CalibrationTime,
      commandCalibrationTime: ADIS16470_IMU.CalibrationTime,
      commandCalibrationWaitTime: float
    ) -> None:
    super().__init__(yawAxis, pitchAxis, rollAxis, port, initCalibrationTime)
    self._commandCalibrationTime = commandCalibrationTime
    self._commandCalibrationWaitTime = commandCalibrationWaitTime

  def calibrate(self) -> None:
    if RobotBase.isReal():
      super().calibrate()

  def set(self, yaw: float) -> None:
    self.setGyroAngle(self.getYawAxis(), yaw)

  def setRobotToZero(self) -> None:
    self.set(0.0)

  def setRobotToField(self, robotPose: Pose2d) -> None:
    self.set(utils.wrapAngle(robotPose.rotation().degrees() + utils.getValueForAlliance(0.0, 180.0)))
  
  def getYaw(self) -> float:
    return utils.wrapAngle(self.getAngle(self.getYawAxis()))
  
  def getPitch(self) -> float:
    return utils.wrapAngle(self.getAngle(self.getPitchAxis()))
  
  def getRoll(self) -> float:
    return utils.wrapAngle(self.getAngle(self.getRollAxis()))
  
  def getHeading(self) -> float:
    return self.getYaw()
  
  def getRotation(self) -> Rotation2d:
    return Rotation2d.fromDegrees(self.getYaw())
  
  def getTurnRate(self) -> float:
    return self.getRate(self.getYawAxis())
  
  def resetCommand(self) -> Command:
    return cmd.runOnce(
      lambda: self.set(0)
    ).ignoringDisable(True).withName("ResetGyro")
  
  def calibrateCommand(self) -> Command:
    return cmd.sequence(
      cmd.runOnce(
        lambda: [
          SmartDashboard.putBoolean("Robot/Sensor/Gyro/IsCalibrating", True),
          self.configCalTime(self._commandCalibrationTime),
          self.calibrate()
        ]
      ),
      cmd.waitSeconds(self._commandCalibrationWaitTime),
      cmd.runOnce(
        lambda: SmartDashboard.putBoolean("Robot/Sensor/Gyro/IsCalibrating", False)
      )
    ).ignoringDisable(True).withName("CalibrateGyro")
  
  def updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Sensor/Gyro/Yaw", self.getYaw())
    SmartDashboard.putNumber("Robot/Sensor/Gyro/Pitch", self.getPitch())
    SmartDashboard.putNumber("Robot/Sensor/Gyro/Roll", self.getRoll())
    SmartDashboard.putNumber("Robot/Sensor/Gyro/TurnRate", self.getTurnRate())