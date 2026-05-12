import math

from autonomous.base_autonomous import BaseAutonomous
from utilities.helpers import FIELD_WIDTH, AutonActions, PathProperty
from utilities.rebuilt_helpers import GamePositions
from wpimath.geometry import Pose2d, Rotation2d


class LeftAggressive(BaseAutonomous):
    MODE_NAME = "Left Agressive 2.0"
    DEFAULT = False

    def __init__(self) -> None:
        super().__init__(
            [
                PathProperty(
                    "LAG1U",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    stashed=False,
                    translation_tolerance=0.8,
                    rotation_tolerance=5,
                    ignore_safety="RBumpAPP",
                ),
                PathProperty("BUMP", bump_target_x=3.0, bump_speed=-3.3),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=4,
                    use_agitate=False,
                ),
                PathProperty(
                    "LINE_UP",
                    auton_action=AutonActions.INTAKE,
                    line_up_to=Pose2d(2.5, 7.51, 0),
                    line_up_y_only=True,
                ),
                PathProperty(
                    "LAG5",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    ignore_safety="RBumpAPP",
                ),
                PathProperty("BUMP", bump_target_x=3.0, bump_speed=-3.3),
                PathProperty(
                    "SNAP_TO_ROTATION", snap_to_rotation=Rotation2d.fromDegrees(143)
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=3,
                    use_agitate=True,
                    agitate_delay=0.0,
                ),
            ]
        )


class LeftSafe(BaseAutonomous):
    MODE_NAME = "Left Safe 2.0"
    DEFAULT = False

    def __init__(self) -> None:
        super().__init__(
            [
                PathProperty(
                    "LSAT1U",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    stashed=False,
                    translation_tolerance=0.6,
                    rotation_tolerance=5,
                    ignore_safety="RBumpAPP",
                ),
                PathProperty("BUMP", bump_target_x=3.2, bump_speed=-3.3),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=4,
                ),
                PathProperty(
                    "LINE_UP",
                    auton_action=AutonActions.INTAKE,
                    line_up_to=Pose2d(2.5, 7.51, 0),
                    line_up_y_only=True,
                ),
                PathProperty(
                    "LAG5",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    translation_tolerance=0.4,
                    rotation_tolerance=2,
                    ignore_safety="RBumpAPP",
                ),
                PathProperty("BUMP", bump_target_x=3.0, bump_speed=-3.3),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=5,
                    use_agitate=True,
                    agitate_delay=0.0,
                ),
            ]
        )


class RightAggressive(BaseAutonomous):
    MODE_NAME = "Right Agressive 2.0"
    DEFAULT = False

    def __init__(self) -> None:
        super().__init__(
            [
                PathProperty(
                    "RAG1U",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    stashed=False,
                    translation_tolerance=0.8,
                    rotation_tolerance=5,
                    ignore_safety="RBumpAPP",
                ),
                PathProperty("BUMP", bump_target_x=3.2, bump_speed=-2.6),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=4,
                    use_agitate=False,
                    wait=4,
                ),
                PathProperty(
                    "LINE_UP",
                    auton_action=AutonActions.INTAKE,
                    line_up_to=Pose2d(2.95, 0.5, 0),
                    line_up_y_only=True,
                ),
                PathProperty(
                    "RAG5",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    ignore_safety="RBUMPAG",
                ),
                PathProperty("BUMP", bump_target_x=3.0, bump_speed=-3.3),
                PathProperty(
                    "SNAP_TO_ROTATION", snap_to_rotation=Rotation2d.fromDegrees(226)
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=3,
                    use_agitate=True,
                    agitate_delay=0.0,
                    wait=3,
                ),
            ]
        )


class RightSafe(BaseAutonomous):
    MODE_NAME = "Right Safe 2.0"
    DEFAULT = False

    def __init__(self) -> None:
        super().__init__(
            [
                PathProperty(
                    "RSAT1U",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    stashed=False,
                    translation_tolerance=0.6,
                    rotation_tolerance=5,
                    ignore_safety="RBumpAPP",
                ),
                PathProperty("BUMP", bump_target_x=3.2, bump_speed=-3.3),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=4,
                    use_agitate=False,
                    # agitate_delay=0.5,
                ),
                PathProperty(
                    "LINE_UP",
                    auton_action=AutonActions.INTAKE,
                    line_up_to=Pose2d(2.5, 0.5, 0),
                    line_up_y_only=True,
                ),
                PathProperty(
                    "RAG5",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    ignore_safety="RBUMPAG",
                ),
                PathProperty("BUMP", bump_target_x=3.0, bump_speed=-3.3),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=5,
                    use_agitate=True,
                    agitate_delay=0.0,
                ),
            ]
        )


class RightCompliant(BaseAutonomous):
    MODE_NAME = "Right Compliant Aggressive1"
    DEFAULT = False

    def __init__(self) -> None:
        super().__init__(
            [
                PathProperty(
                    "Nothing",
                    wait=3,
                    stashed=True,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.48, 0.48, 0),
                    stashed=True,
                ),
                PathProperty(
                    # 5.0  seconds
                    "RAG1U",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    translation_tolerance=0.4,
                    rotation_tolerance=2,
                    ignore_safety="RBumpAPP",
                ),
                PathProperty("BUMP", bump_target_x=3.0, bump_speed=-3.3),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(1.1, 1.2, 4),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=5,
                    use_agitate=False,
                    agitate_delay=0.0,
                ),
                PathProperty(
                    "LINE_UP",
                    auton_action = AutonActions.INTAKE,
                    line_up_to=Pose2d(2.5, 0.5, 0),
                    stashed=False,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.48, 0.5, 0),
                    stashed=False,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(8.3, 0.58, 0),
                    stashed=False,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(8.3, 0.58, 1.5),
                    stashed=False,
                ),
                # PathProperty("LINE_UP", line_up_to=Pose2d(2.08, 2.3, math.pi)),
                # PathProperty("LINE_UP", line_up_to=Pose2d(1.7, 5.9, math.pi)),
                #     PathProperty(
                #         "LINE_UP",
                #         line_up_to=Pose2d(1, 5.9, math.pi),
                #         auton_action=AutonActions.INTAKE,
                #     ),
                #     PathProperty("LINE_UP", line_up_to=Pose2d(2.08, 5.9, math.pi)),
                #     PathProperty(
                #         "NOTHING",
                #         auton_action=AutonActions.SCORE,
                #         parallel=False,
                #         reverse=False,
                #         lock_heading=False,
                #         stationary_action_time=5,
                #         use_agitate=True,
                #         agitate_delay=0.0,
                #     ),
            ],
            manual_start_pose=Pose2d(3.5, 0.5, 0),
        )
class Leftcompliant(BaseAutonomous):
    MODE_NAME = "Left Compliant Aggressive1"
    DEFAULT = False

    def __init__(self) -> None:
        super().__init__(
            [
                PathProperty(
                    "Nothing",
                    wait=4,
                    stashed=True,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.48, (8.0137 - 0.48), 0),
                    stashed=True,
                ),
                PathProperty(
                    # 5.0  seconds
                    "LAG1U",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    translation_tolerance=0.4,
                    rotation_tolerance=2,
                    ignore_safety="RBumpAPP",
                ),
                PathProperty("BUMP", bump_target_x=3.0, bump_speed=-3.3),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(1.6, (8.0137 - 1.0), Rotation2d.fromDegrees(135)),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=5,
                    use_agitate=False,
                    agitate_delay=0.0,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(2.5, (8.0137 - 0.5), 0),
                    stashed=False,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.48, (8.0137 - 0.5), 0),
                    stashed=False,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(8.3, (8.0137 - 0.54), 0),
                    stashed=False,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(8.3, (8.0137 - 0.54), -1.5),
                    stashed=False,
                ),
                # PathProperty("LINE_UP", line_up_to=Pose2d(2.08, 2.3, math.pi)),
                # PathProperty("LINE_UP", line_up_to=Pose2d(1.7, 5.9, math.pi)),
                #     PathProperty(
                #         "LINE_UP",
                #         line_up_to=Pose2d(1, 5.9, math.pi),
                #         auton_action=AutonActions.INTAKE,
                #     ),
                #     PathProperty("LINE_UP", line_up_to=Pose2d(2.08, 5.9, math.pi)),
                #     PathProperty(
                #         "NOTHING",
                #         auton_action=AutonActions.SCORE,
                #         parallel=False,
                #         reverse=False,
                #         lock_heading=False,
                #         stationary_action_time=5,
                #         use_agitate=True,
                #         agitate_delay=0.0,
                #     ),
            ],
            manual_start_pose=Pose2d(3.5, (8.0137 - 0.5), 0),
        )



# class LeftCompliant(BaseAutonomous):
#     MODE_NAME = "Left Compliant Aggressive"
#     DEFAULT = False

#     def __init__(self) -> None:
#         super().__init__(
#             [
#                 PathProperty(
#                     "LINE_UP", line_up_to=Pose2d(4.48, (8.0137 - 0.48), 0), stashed=True
#                 ),
#                 PathProperty("NOTHING", wait=2),
#                 PathProperty(
#                     "LAG1U",
#                     auton_action=AutonActions.INTAKE,
#                     parallel=True,
#                     reverse=False,
#                     lock_heading=False,
#                     translation_tolerance=0.4,
#                     rotation_tolerance=2,
#                     ignore_safety="RBumpAPP",
#                 ),
#                 PathProperty("BUMP", bump_target_x=3.0, bump_speed=-3.3),
#                 PathProperty(
#                     "NOTHING",
#                     auton_action=AutonActions.SCORE,
#                     parallel=False,
#                     reverse=False,
#                     lock_heading=False,
#                     stationary_action_time=5,
#                     use_agitate=False,
#                     agitate_delay=0.0,
#                 ),
#                 PathProperty(
#                     "LINE_UP", line_up_to=Pose2d(2.08, (8.0137 - 2.3), math.pi)
#                 ),
#                 PathProperty(
#                     "LINE_UP", line_up_to=Pose2d(1.7, (8.0137 - 2.1), math.pi)
#                 ),
#                 PathProperty(
#                     "LINE_UP",
#                     line_up_to=Pose2d(1, (8.0137 - 2.1), math.pi),
#                     auton_action=AutonActions.INTAKE,
#                 ),
#                 PathProperty("LINE_UP", line_up_to=Pose2d(1, (8.0137 - 1), math.pi)),
#                 PathProperty(
#                     "NOTHING",
#                     auton_action=AutonActions.SCORE,
#                     parallel=False,
#                     reverse=False,
#                     lock_heading=False,
#                     stationary_action_time=5,
#                     use_agitate=True,
#                     agitate_delay=0.0,
#                 ),
#             ],
#             manual_start_pose=Pose2d(3.5, (8.0137 - 0.5), 0),
#         )


class RightDefense(BaseAutonomous):
    MODE_NAME = "Right Defense"

    def __init__(self) -> None:
        super().__init__(
            [
                PathProperty("NOTHING", wait=3, stashed=True),
                PathProperty("LINE_UP", line_up_to=Pose2d(6.00, 0.54, 0), stashed=True),
                PathProperty(
                    "RD1",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    translation_tolerance=0.2,
                    rotation_tolerance=2,
                ),
                PathProperty("NOTHING", wait=1),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(3.35, 0.54, math.pi),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(2.0, 0.54, math.pi),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(1.1, 1.1, math.pi),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=5,
                    use_agitate=False,
                    agitate_delay=0.0,
                ),
            ],
            manual_start_pose=Pose2d(3.5, (0.5), 0),
        )


class LeftDefense(BaseAutonomous):
    MODE_NAME = "Left Defense"

    def __init__(self) -> None:
        super().__init__(
            [
                PathProperty("NOTHING", wait=3, stashed=True),
                PathProperty(
                    "LINE_UP", line_up_to=Pose2d(6.00, (8.0137 - 0.54), 0), stashed=True
                ),
                PathProperty(
                    "LD1",
                    auton_action=AutonActions.INTAKE,
                    parallel=True,
                    reverse=False,
                    lock_heading=False,
                    translation_tolerance=0.2,
                    rotation_tolerance=2,
                ),
                PathProperty("NOTHING", wait=1),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(3.35, (8.0137 - 0.54), math.pi),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(2.0, (8.0137 - 0.54), math.pi),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(1.1, (8.0137 - 1.1), math.pi),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    parallel=False,
                    reverse=False,
                    lock_heading=False,
                    stationary_action_time=5,
                    use_agitate=False,
                    agitate_delay=0.0,
                ),
            ],
            manual_start_pose=Pose2d(3.5, (8.0137 - 0.5), 0),
        )


class RightDoubleTrench(BaseAutonomous):
    MODE_NAME = "Right Double Trench"
    DEFAULT = False

    def __init__(self):
        super().__init__(
            paths=[
                PathProperty(
                    "RTRENCH1", auton_action=AutonActions.INTAKE, parallel=True
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.0, 0.56, math.pi),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(2.5, 1, Rotation2d.fromDegrees(225)),
                    auton_action=AutonActions.NOTHING,
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    stationary_action_time=3.5,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(2.5, 0.56, 0),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.0, 0.56, 0),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "RTRENCH1", auton_action=AutonActions.INTAKE, parallel=True
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.0, 0.56, math.pi),
                    auton_action=AutonActions.INTAKE,
                    line_up_y_only=True,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(2.5, 1, Rotation2d.fromDegrees(225)),
                    auton_action=AutonActions.NOTHING,
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    stationary_action_time=3.5,
                ),
            ],
        )


class LeftDoubleTrench(BaseAutonomous):
    MODE_NAME = "Left Double Trench"
    DEFAULT = False

    def __init__(self):
        super().__init__(
            paths=[
                PathProperty(
                    "LTRENCH1", auton_action=AutonActions.INTAKE, parallel=True
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.0, (8.0137 - 0.56), math.pi),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(2.5, (8.0137 - 1), Rotation2d.fromDegrees(225)),
                    auton_action=AutonActions.NOTHING,
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    stationary_action_time=3.5,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(2.5, (8.0137 - 0.56), 0),
                    auton_action=AutonActions.INTAKE,
                    line_up_y_only=True,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.0, (8.0137 - 0.56), 0),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "LTRENCH1", auton_action=AutonActions.INTAKE, parallel=True
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(4.0, (8.0137 - 0.56), math.pi),
                    auton_action=AutonActions.INTAKE,
                ),
                PathProperty(
                    "LINE_UP",
                    line_up_to=Pose2d(2.5, (8.0137 - 1), Rotation2d.fromDegrees(225)),
                    auton_action=AutonActions.NOTHING,
                ),
                PathProperty(
                    "NOTHING",
                    auton_action=AutonActions.SCORE,
                    stationary_action_time=3.5,
                ),
            ],
            # manual_start_pose=Pose2d(4.48, (8.0137 - 0.48), 0)
        )


# class BackAndShoot(BaseAutonomous):
#     MODE_NAME = "Back And Shoot"
#     DEFAULT = False

#     def __init__(self) -> None:
#         super().__init__(
#             [
#                 PathProperty(
#                     "LINE_UP",
#                     auton_action=AutonActions.NOTHING,
#                     line_up_to=Pose2d(2.1, 3.7, math.pi),
#                     translation_tolerance=0.8,
#                     rotation_tolerance=5,
#                 ),
#                 PathProperty(
#                     "NOTHING",
#                     auton_action=AutonActions.SCORE,
#                     parallel=False,
#                     reverse=False,
#                     lock_heading=False,
#                     stationary_action_time=20,
#                 ),
#             ],
#             manual_start_pose=Pose2d(3.6, 2.4, 0),
#         )
