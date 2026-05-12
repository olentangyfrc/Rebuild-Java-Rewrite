from enum import Enum


class AutonActions(Enum):
    """
    Possible actions that can be done on each route in an autonomous routine

    Routes only support having one auton action per each
    """

    INTAKE = "Intake"
    SCORE = "Score"
    CLIMB = "Climb"
    NOTHING = "Nothing"
    DEPLOY_INTAKE = "Deploy Intake"


class DrivetrainStates(Enum):
    ACTIVE_SNAP = "Active Snap"
    PASSIVE_SNAP = "Passive Snap"
    LINE_UP = "Line up"
    LOCK = "Lock"
    POINT_AT_POSE = "Point at pose"
    DRIVER_ASSIST = "Driver Assist"
    SNAKE = "SNAKE"


class ShooterStates(Enum):
    IDLE = "Idle"
    SPIN_UP = "Spin Up"
    SHOOT = "Shoot"
    EJECT = "Eject"
    COOLDOWN = "Cooldown"
    WARM_UP = "Warm Up"


class SerializerStates(Enum):
    IDLE = "Idle"
    FORWARD = "Forward"
    REVERSE = "Reverse"
    SLOW_REVERSE = "Slow Reverse"
    PASS_FORWARD = "Pass Forward"


class ClimberStates(Enum):
    HOLD = "Hold"
    RELEASE = "Release"
    CLIMB = "Climb"
    IDLE = "Idle"


class IntakeStates(Enum):
    IDLE = "Idle"
    INTAKE = "Intake"
    DEPLOYED = "Deployed"
    EJECT = "EJECT"
    PUSH_FORWARD = "PUSH FORWARD"
    STASHED = "STASHED"
    EMERGENCY_STASH = "Emergency Stash"
    RETRACT = "Retract"
    AGITATE = "Agitate Forward"
    AGITATE_OSCILLATE = "Agitate Oscillate"
    INTAKE_PASS = "Intake Pass"


class SuperStructureStates(Enum):
    IDLE = "IDLE"
    INTAKE = "INTAKE"
    SHOOT = "SHOOT"
    EJECT = "EJECT"
    UNJAM = "UNJAM"
    STASH = "STASH"
    PASS = "PASS"
    DEPLOYED_CLIMBER = "DEPLOYED_CLIMBER"
    CLIMB = "CLIMB"
