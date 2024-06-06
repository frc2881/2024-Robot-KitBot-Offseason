from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpimath import units
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.pathfinding import PathConstraints
if TYPE_CHECKING: from commands.game_commands import GameCommands
from classes import AutoPath
import constants

class AutoCommands:
  def __init__(
      self, 
      gameCommands: "GameCommands"
    ) -> None:
    self.gameCommands = gameCommands

  def _getPath(self, autoPath: AutoPath) -> PathPlannerPath:
    return constants.Game.Auto.kPaths.get(autoPath)
  
  def _move(self, path: PathPlannerPath) -> Command:
    return AutoBuilder.pathfindThenFollowPath(
      path, constants.Subsystems.Drive.kPathFindingConstraints
    ).withName("AutoMoveWithPath")
  
   # ######################################################################
   # ################################ AUTOS ###############################
   # ######################################################################

  def test(self) -> Command:
    return cmd.sequence(
      AutoBuilder.pathfindThenFollowPath(
        constants.Game.Auto.kPaths.get(AutoPath.Test),
        PathConstraints(1.5, 1.5, units.degreesToRadians(270), units.degreesToRadians(360))
      )
    ).withName("Test")
  