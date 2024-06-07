from commands2 import Subsystem, Command, cmd
from rev import CANSparkMax, CANSparkLowLevel
import constants

class LauncherSubsystem(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Launcher

    self._launchWheel = CANSparkMax(self._constants.kLauncherID, CANSparkLowLevel.MotorType.kBrushless)
    self._feedWheel = CANSparkMax(self._constants.kFeederID, CANSparkLowLevel.MotorType.kBrushless)

    self._launchWheel.setInverted(True)
    self._feedWheel.setInverted(True)

    self._launchWheel.setSmartCurrentLimit(self._constants.kLauncherCurrentLimit)
    self._feedWheel.setSmartCurrentLimit(self._constants.kFeedCurrentLimit)

  def runIntakeCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._setFeedWheel(self._constants.kIntakeFeederSpeed),
        self._setLaunchWheel(self._constants.kIntakeLauncherSpeed)
      ],
      lambda: self._stop()
    ).withName("RunIntake")

  def runLauncherCommand(self) -> Command:
    return self.runOnce(
      lambda: self._setLaunchWheel(self._constants.kLauncherSpeed)
    ).andThen(
      cmd.waitSeconds(self._constants.kLauncherDelay)
    ).andThen(
      cmd.runOnce(lambda: self._setFeedWheel(self._constants.kLaunchFeederSpeed))
    ).andThen(
      cmd.waitSeconds(self._constants.kLauncherDelay)
    ).finallyDo(
      lambda end: self._stop()
    ).withName("RunLauncher")

  #  An accessor method to set the speed (technically the output percentage) of the launch wheel
  def _setLaunchWheel(self, speed: float) -> None:
    self._launchWheel.set(speed)

  #  An accessor method to set the speed (technically the output percentage) of the feed wheel
  def _setFeedWheel(self, speed: float) -> None:
    self._feedWheel.set(speed)

  #  A helper method to stop both wheels. You could skip having a method like this and call the individual accessors with speed = 0 instead
  def _stop(self) -> None:
    self._launchWheel.set(0)
    self._feedWheel.set(0)
