from enum import Enum, auto
from dataclasses import dataclass

class Alliance(Enum):
  Red = 0
  Blue = 1

class RobotMode(Enum):
  Disabled = auto()
  Auto = auto()
  Teleop = auto()
  Test = auto()

class RobotState(Enum):
  Disabled = auto()
  Enabled = auto()
  EStopped = auto()

class MotorDirection(Enum):
  Stopped = auto()
  Forward = auto()
  Reverse = auto()

class SwerveModuleLocation(Enum):
  FrontLeft = auto()
  FrontRight = auto()
  RearLeft = auto()
  RearRight = auto()

class DriveOrientation(Enum):
  Field = auto()
  Robot = auto()

class DriveSpeedMode(Enum):
  Competition = auto()
  Training = auto()

class DriveLockState(Enum):
  Unlocked = auto()
  Locked = auto()

class DriveDriftCorrection(Enum):
  Enabled = auto()
  Disabled = auto()

class ControllerRumbleMode(Enum):
  Both = auto()
  Driver = auto()
  Operator = auto()

class ControllerRumblePattern(Enum):
  Short = auto()
  Long = auto()

@dataclass
class PIDConstants:
  P: float
  I: float
  D: float
  FF: float

@dataclass
class ObjectTargetInfo:
  yaw: float
  area: float
