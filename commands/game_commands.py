from typing import TYPE_CHECKING
from commands2 import Command, cmd
if TYPE_CHECKING: from robot_container import RobotContainer
from lib import utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
import constants

class GameCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self.robot = robot

  def alignRobotToTargetCommand(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self.robot.driveSubsystem.alignToTargetCommand(
          lambda: self.robot.localizationSubsystem.getPose(), 
          lambda: self.robot.localizationSubsystem.getTargetYaw()
        ).withTimeout(utils.getValueForRobotMode(2.0, float("inf"))),
        cmd.either(self.rumbleControllersCommand(ControllerRumbleMode.Operator, ControllerRumblePattern.Short), cmd.none(), lambda: not utils.isAutonomousMode())
      ),
      cmd.either(self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short), cmd.none(), lambda: not utils.isAutonomousMode())
    ).withName("AlignRobotToTarget")
  
  def rumbleControllersCommand(self, mode: ControllerRumbleMode, pattern: ControllerRumblePattern) -> Command:
    return cmd.parallel(
      self.robot.driverController.rumbleCommand(pattern)
      .onlyIf(lambda: mode == ControllerRumbleMode.Driver or mode == ControllerRumbleMode.Both),
      self.robot.operatorController.rumbleCommand(pattern)
      .onlyIf(lambda: mode == ControllerRumbleMode.Operator or mode == ControllerRumbleMode.Both)
    ).withName("RumbleControllers")