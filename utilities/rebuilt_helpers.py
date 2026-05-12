from collections.abc import Iterable
from dataclasses import dataclass
from enum import Enum
import math

import wpimath.units as units
from wpilib import DriverStation, Field2d, Color
from wpimath.geometry import (
    Ellipse2d,
    Pose2d,
    Rectangle2d,
    Rotation2d,
    Transform2d,
    Translation2d,
)

from utilities import data
from utilities.helpers import (
    flip_pose,
    get_angle_between_poses,
    FIELD_WIDTH,
    FIELD_LENGTH,
)


class GamePositions(Enum):
    HUB = Pose2d(4.6, 4, 0)
    TRENCH_DEPOT = Pose2d(4.6, 7.44, 0)
    TRENCH_NON_DEPOT = Pose2d(4.6, 0.56, 0)
    SLOW_ZONE_TARGET = Pose2d(2.5, 3, 0)
    TOWER_CENTER = Pose2d(1, 3.753, 0)
    RIGHT_TRENCH_APPROACH = Pose2d(3.00, 0.5, 0)
    BUMP_NON_DEPOT = Pose2d(4.6, 2.5, 0)
    BUMP_DEPOT = Pose2d(4.6, 5.5, 0)


class ShiftScheduler:
    initialized = False
    _active_times: list[units.seconds] = []
    _active_shifts: list[str] = []
    _current_index = 0

    _alliance_color_map = {
        DriverStation.Alliance.kBlue: Color.kBlue.hexString(),
        DriverStation.Alliance.kRed: Color.kRed.hexString(),
    }

    _shift_to_time_map = {
        "Shift 1": 130,
        "Shift 2": 105,
        "Shift 3": 80,
        "Shift 4": 55,
        "Endgame": 30,
    }

    _current_time = 0.0

    def __init__(self) -> None:
        raise SyntaxError("Treat as a singleton")

    @staticmethod
    def _filter_current_alliance() -> str:
        alliance = DriverStation.getAlliance()
        if not alliance:
            return ""
        return "r" if alliance == DriverStation.Alliance.kRed else "b"

    @staticmethod
    def schedule_shift_times() -> None:
        msg = DriverStation.getGameSpecificMessage().lower()
        current_alliance = ShiftScheduler._filter_current_alliance()

        if msg not in ["r", "b"] or not current_alliance:
            return

        ShiftScheduler.initialized = True
        if current_alliance == msg:  # won auto
            ShiftScheduler._active_shifts = ["Shift 2", "Shift 4", "Endgame"]
        else:
            ShiftScheduler._active_shifts = ["Shift 1", "Shift 3", "Endgame"]

        ShiftScheduler._active_times = sorted(
            [
                ShiftScheduler._shift_to_time_map.get(shift, 0.0)
                for shift in ShiftScheduler._active_shifts
            ],
            reverse=True,
        )

    @staticmethod
    def check_active_shift_upcoming(margin: units.seconds = 5) -> bool:
        if not ShiftScheduler.initialized:
            return False

        current_time = DriverStation.getMatchTime()

        if current_time <= ShiftScheduler._active_times[ShiftScheduler._current_index]:
            if ShiftScheduler._current_index < len(ShiftScheduler._active_times) - 1:
                ShiftScheduler._current_index += 1

        return (
            current_time - ShiftScheduler._active_times[ShiftScheduler._current_index]
            <= margin
        )

    @staticmethod
    def check_shift_change_upcoming(margin: units.seconds) -> bool:
        if ShiftScheduler.get_current_shift() == "endgame":
            return False
        else:
            return ShiftScheduler.get_time_to_next_shift() <= margin

    @staticmethod
    def get_current_shift() -> str:
        if not ShiftScheduler.initialized:
            return ""

        time = DriverStation.getMatchTime()

        if DriverStation.isAutonomous():
            return "Auton"
        elif time <= 30:
            return "Endgame"
        elif time <= 55:
            return "Shift 4"
        elif time <= 80:
            return "Shift 3"
        elif time <= 105:
            return "Shift 2"
        elif time <= 130:
            return "Shift 1"
        return "Transition Shift"

    @staticmethod
    def hub_is_active() -> bool:
        if not ShiftScheduler.initialized:
            return False
        return ShiftScheduler.get_current_shift() in ShiftScheduler._active_shifts

    @staticmethod
    def get_time_to_next_shift() -> float:
        if not ShiftScheduler.initialized:
            return -1.0

        if ShiftScheduler.get_current_shift() == "Endgame":
            return 0.0
        elif ShiftScheduler.get_current_shift() == "Transition Shift":
            return DriverStation.getMatchTime() - 130
        else:
            return DriverStation.getMatchTime() - (
                ShiftScheduler._shift_to_time_map.get(
                    ShiftScheduler.get_current_shift(), 0.0
                )
                - 25
            )  # -25 to get the next shift

    @staticmethod
    def get_active_alliance() -> str:
        alliance = DriverStation.getAlliance()
        current_shift = ShiftScheduler.get_current_shift()

        if (
            current_shift in ["Auton", "Transition Shift", "Endgame"]
            or not alliance
            or not current_shift
        ):
            return Color.kGray.hexString()
        elif current_shift in ShiftScheduler._active_shifts:
            return ShiftScheduler._alliance_color_map.get(
                alliance, Color.kGray.hexString()
            )
        else:
            return ShiftScheduler._alliance_color_map.get(
                DriverStation.Alliance.kBlue
                if alliance == DriverStation.Alliance.kRed
                else DriverStation.Alliance.kRed,
                Color.kGray.hexString(),
            )


class SoftWall:
    def __init__(
        self,
        bounding_boxes: Rectangle2d | Ellipse2d | Iterable[Rectangle2d | Ellipse2d],
        names: str | Iterable[str],
        enable: bool = True,
    ) -> None:
        self.bounding_boxes = bounding_boxes
        self.names = names
        self._enabled = enable
        self._is_iterable = not isinstance(bounding_boxes, (Rectangle2d, Ellipse2d))

    def contains(self, pose: Pose2d) -> bool:
        if not self._enabled:
            return False
        if self._is_iterable:
            return any(b.contains(pose.translation()) for b in self.bounding_boxes)  # type: ignore
        return self.bounding_boxes.contains(pose.translation())  # type: ignore

    def update_on_field(self, field: Field2d):
        if self._is_iterable:
            for bound, name in zip(self.bounding_boxes, self.names):  # type: ignore
                pos = bound.center() if self._enabled else Pose2d(100, 100, 0)
                field.getObject(name).setPose(pos)
        else:
            pos = self.bounding_boxes.center() if self._enabled else Pose2d(100, 100, 0)  # type: ignore
            field.getObject(self.names).setPose(pos)  # type: ignore

    def disable(self) -> None:
        self._enabled = False

    def enable(self) -> None:
        self._enabled = True

    def is_enabled(self) -> bool:
        return self._enabled


class SoftWalls(Enum):
    TRENCH = SoftWall(
        (
            Rectangle2d(GamePositions.TRENCH_DEPOT.value, 1, 1.5),
            Rectangle2d(GamePositions.TRENCH_NON_DEPOT.value, 1, 1.5),
            Rectangle2d(flip_pose(GamePositions.TRENCH_DEPOT.value, True), 1, 1.5),
            Rectangle2d(flip_pose(GamePositions.TRENCH_NON_DEPOT.value, True), 1, 1.5),
        ),
        (
            "Trench Depot",
            "Trench Non Depot",
            "Opposite Trench Depot",
            "Opposite Trench Non Depot",
        ),
    )
    TRENCH_ASSIST = SoftWall(
        (
            Rectangle2d(GamePositions.TRENCH_DEPOT.value, 7, 3),
            Rectangle2d(GamePositions.TRENCH_NON_DEPOT.value, 7, 3),
            Rectangle2d(flip_pose(GamePositions.TRENCH_DEPOT.value, True), 7, 3),
            Rectangle2d(flip_pose(GamePositions.TRENCH_NON_DEPOT.value, True), 7, 3),
        ),
        (
            "Trench Depot",
            "Trench Non Depot",
            "Opposite Trench Depot",
            "Opposite Trench Non Depot",
        ),
    )
    SLOW_ZONE = SoftWall(
        Rectangle2d(GamePositions.SLOW_ZONE_TARGET.value, 2, 2), "Precision Slow Zone"
    )
    BUMP_EJECT_ZONE = SoftWall(
        (
            Rectangle2d(
                GamePositions.BUMP_DEPOT.value.transformBy(Transform2d(1, 0, 0)), 1, 2
            ),
            Rectangle2d(
                GamePositions.BUMP_NON_DEPOT.value.transformBy(Transform2d(1, 0, 0)),
                1,
                2,
            ),
        ),
        ("Bump Eject Zone Depot", "Bump Eject Zone Non Depot"),
    )


class Interpolation:
    @dataclass
    class DataPoint:
        distance: units.meters
        angle: units.radians
        rpm: units.revolutions_per_minute

    def __init__(self):
        # data_points = data.low_ceiling_interpolation_data.copy()
        data_points = data.high_ceiling_interpolation_data.copy()
        data_points.sort(key=lambda x: x[0])  # sort by ascending distance

        self.interpolation_data = [
            Interpolation.DataPoint(distance, math.radians(angle), rpm)
            for distance, angle, rpm in data_points
        ]

    def _get_values_for_interpolation(
        self, distance: units.meters
    ) -> tuple[DataPoint | None, DataPoint | None]:
        data = self.interpolation_data

        if distance > data[-1].distance:
            return data[-2], data[-1]
        if distance < data[0].distance:
            return data[0], data[1]

        for index, point in enumerate(data):
            if distance == point.distance:
                return point, None
            elif distance < point.distance:
                return data[index - 1], point

        return None, None

    def get_interpolated_values(
        self, distance: units.meters
    ) -> tuple[units.radians, units.revolutions_per_minute] | None:
        low_point, high_point = self._get_values_for_interpolation(distance)

        if high_point is None and low_point is None:
            return None
        elif high_point is None and low_point is not None:
            return (low_point.angle, low_point.rpm)

        if low_point.distance == distance:
            return (low_point.angle, low_point.rpm)
        elif high_point.distance == distance:
            return (high_point.angle, high_point.rpm)

        target_theta = low_point.angle + (
            (high_point.angle - low_point.angle)
            / (high_point.distance - low_point.distance)
        ) * (distance - low_point.distance)
        target_rpm = low_point.rpm + (
            (high_point.rpm - low_point.rpm)
            / (high_point.distance - low_point.distance)
        ) * (distance - low_point.distance)

        return (target_theta, math.floor(target_rpm))


def get_pass_angle(pose: Pose2d) -> Rotation2d:
    if pose.X() < 11.2:
        if pose.Y() < 3.45:
            return Rotation2d(
                get_angle_between_poses(pose, Pose2d(3.2, 1.5, 0)) + math.pi
            )
        elif pose.Y() > 4.65:
            return Rotation2d(
                get_angle_between_poses(pose, Pose2d(3.2, FIELD_WIDTH - 1.5, 0))
                + math.pi
            )
        elif pose.Y() < 4:
            return Rotation2d(
                get_angle_between_poses(pose, Pose2d(3.2, 1.4, 0)) + math.pi
            )
        else:
            return Rotation2d(
                get_angle_between_poses(pose, Pose2d(3.2, 6.5, 0)) + math.pi
            )
    else:
        if pose.Y() < 3.45:
            return Rotation2d(
                get_angle_between_poses(pose, Pose2d(5.67, 2.5, 0)) + math.pi
            )
        elif pose.Y() > 4.65:
            return Rotation2d(
                get_angle_between_poses(pose, Pose2d(5.67, FIELD_WIDTH - 2.5, 0))
                + math.pi
            )
        elif pose.Y() < 4:
            return Rotation2d(
                get_angle_between_poses(pose, Pose2d(10, 1.4, 0)) + math.pi
            )
        else:
            return Rotation2d(
                get_angle_between_poses(pose, Pose2d(10, 6.5, 0)) + math.pi
            )


def get_pass_distance(pose: Pose2d) -> units.meters:
    if pose.X() < 11.2:
        if pose.Y() < 3.45:
            return pose.translation().distance(Translation2d(3.2, 1.5))
        elif pose.Y() > 4.65:
            return pose.translation().distance(Translation2d(3.2, FIELD_WIDTH - 1.5))
        elif pose.Y() < 4:
            return pose.translation().distance(Translation2d(3.2, 1.4))
        else:
            return pose.translation().distance(Translation2d(3.2, 6.5))
    else:  # For full field passing, it passes slighltly after the bump so the fuel bounce over it. It is not possible to pass to our field without browning out.
        if pose.Y() < 3.45:
            return pose.translation().distance(Translation2d(5.67, 2.5))
        elif pose.Y() > 4.65:
            return pose.translation().distance(Translation2d(5.67, FIELD_WIDTH - 2.5))
        elif pose.Y() < 4:
            return pose.translation().distance(Translation2d(10, 1.4))
        else:
            return pose.translation().distance(Translation2d(10, 6.5))
