import math
from collections.abc import Collection, Sequence
from logging import Logger
from typing import Any

from magicbot import tunable
from ntcore import NetworkTableInstance
from wpimath import units

# from ntcore.util import ChooserControl
# from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import (
    DriverStation,
    Field2d,
    SendableChooser,
    SmartDashboard,
    Timer,
)
from wpimath.geometry import Pose2d

from components.drivetrain import Drivetrain
from utilities.configs import VisionConfig
from utilities.elasticlib import Notification, NotificationManager
from utilities.helpers import get_struct_string
from utilities.IO import VisionIO
from utilities.vision_utils import (
    VisionMeasurement,
    get_recent_vision_measurements,
    # get_vision_measurements_from_photoncamera,
    set_robot_orientation,
)

# if RobotBase.isSimulation():
# from photonlibpy import PhotonCamera


class Vision:
    drivetrain: Drivetrain
    logger: Logger

    send_yaw_rate = tunable(True)

    def __init__(self, config: VisionConfig) -> None:
        """
        Initializes the vision subsystem by creating a networktables instance with all given limelights.
        """
        self.io = VisionIO()
        self.config = config
        self.camera_len = (
            len(self.config.camera_names)
            if isinstance(self.config.camera_names, Collection)
            else 1
        )

        if isinstance(self.config.camera_names, str):
            self.config.camera_names = [self.config.camera_names]

        self.mt1_measurement_tracker: dict[str, list[VisionMeasurement]] = {
            name: [] for name in config.camera_names
        }

        self.mt2_measurement_tracker: dict[str, list[VisionMeasurement]] = {
            name: [] for name in config.camera_names
        }

        self.vision_field = Field2d()
        self.vision_field.setRobotPose(Pose2d())

        self.mt2_nt_topic = "botpose_orb_wpiblue"
        self.mt1_nt_topic = "botpose_wpiblue"

        # if RobotBase.isSimulation():
        #     self.apriltag_layout = AprilTagFieldLayout.loadField(
        #         AprilTagField.k2025ReefscapeAndyMark
        #     )

        #     self.sim_photon_camera_lookup = {
        #         name: PhotonCamera(name) for name in self.config.camera_names
        #     }

        self._set_up_logging()

    def _set_up_logging(self):
        self.disable_chooser = SendableChooser()
        self.disable_chooser.setDefaultOption("None", None)
        self.disable_chooser.addOption("all", "all")

        for name in self.mt2_measurement_tracker:
            self.disable_chooser.addOption(name, name)

        self.disable_chooser.onChange(lambda x: send(x))

        SmartDashboard.putData("Disable Chooser", self.disable_chooser)
        SmartDashboard.putData("Vision Field", self.vision_field)

        def send(x: Any):
            nonlocal self  # Fixes some scope issues

            if x not in [None, "all"]:
                NotificationManager.add_unconditional_notification(
                    Notification(
                        title="Limelight disabled",
                        description=f"{x} will not feed any more pose info",
                    )
                )
            elif x == "all":
                NotificationManager.add_unconditional_notification(
                    Notification(
                        title="All limelights disabled",
                        description="Pose info will solely be based off odometery now",
                    )
                )

    def setup(self) -> None:
        for name in self.config.camera_names:
            set_robot_orientation(
                name,
                self.drivetrain.get_gyro_rotation().degrees(),
                self.drivetrain.get_gyro_velocity() * self.send_yaw_rate,
                0,  # Pitch
            )

        for name in self.config.camera_names:
            NetworkTableInstance.getDefault().getTable(name).getEntry(
                "imumode_set"
            ).setInteger(0)  # 2 is for use internal imu

        self.start_rewind()

    def slow_down_processing(self) -> None:
        # To manage temperature for the limelight.
        # for name in self.config.camera_names:
        #     NetworkTableInstance.getDefault().getTable(name).getEntry(
        #         "throttle_set"
        #     ).setInteger(150)
        pass

    def speed_up_processing(self) -> None:
        for name in self.config.camera_names:
            NetworkTableInstance.getDefault().getTable(name).getEntry(
                "throttle_set"
            ).setInteger(0)

    def start_rewind(self) -> None:
        for name in ["limelight-left", "limelight-right"]:
            table = NetworkTableInstance.getDefault().getTable(name)
            table.getEntry("rewind_enable_set").setDouble(1.0)
            # table.getEntry("rewind_enable_set").setDouble(0.0)
            table.getEntry("capture_rewind").setDoubleArray(
                [0, 0]  # resetting rewind counter to 0
            )

    def capture_rewind(self, duration: units.seconds = 165.0) -> None:
        if not hasattr(self, "rewind_capture_counter"):
            self.rewind_capture_counter = 0
        else:
            self.rewind_capture_counter += 1

        for name in ["limelight-left", "limelight-right"]:
            table = NetworkTableInstance.getDefault().getTable(name)
            # table.getEntry("rewind_enable_set").setDouble(0.0)
            table.getEntry("capture_rewind").setDoubleArray(
                [self.rewind_capture_counter, duration]
            )

    def execute(self) -> None:
        """
        Run every auton and teleop periodic cycle. Feeds vision measurements to the drivetrain to update odometry
        """
        yaw_rate = self.drivetrain.get_gyro_velocity()
        for i, camera_name in enumerate(self.config.camera_names):
            if (
                camera_name == self.disable_chooser.getSelected()
                or self.disable_chooser.getSelected() == "all"
            ):
                continue

            mt2_tracker = self.mt2_measurement_tracker[camera_name]
            mt1_tracker = self.mt1_measurement_tracker[camera_name]

            set_robot_orientation(
                camera_name,
                self.drivetrain.get_gyro_rotation().degrees(),
                yaw_rate * self.send_yaw_rate,
                0,
            )

            recent_mt2_measurements = get_recent_vision_measurements(
                camera_name, self.mt2_nt_topic
            )
            recent_mt1_measurements = get_recent_vision_measurements(
                camera_name, self.mt1_nt_topic
            )

            self.drivetrain.add_vision_measurement(
                recent_mt2_measurements, recent_mt1_measurements
            )

            mt2_tracker.extend(recent_mt2_measurements)
            mt1_tracker.extend(recent_mt1_measurements)

            for i, measurement in enumerate(mt2_tracker):
                if (
                    Timer.getFPGATimestamp() - measurement.timestamp
                    < self.config.time_delay
                ):
                    self.mt2_measurement_tracker[camera_name] = mt2_tracker[i:]
                    break

            for i, measurement in enumerate(mt1_tracker):
                if (
                    Timer.getFPGATimestamp() - measurement.timestamp
                    < self.config.time_delay
                ):
                    self.mt1_measurement_tracker[camera_name] = mt1_tracker[i:]
                    break

            if (
                self.config.max_frame_count is not None
                and len(mt2_tracker) > self.config.max_frame_count
            ):
                index = len(mt2_tracker) - self.config.max_frame_count
                self.mt2_measurement_tracker[camera_name] = mt2_tracker[index:]

            if (
                self.config.max_frame_count is not None
                and len(mt1_tracker) > self.config.max_frame_count
            ):
                index = len(mt1_tracker) - self.config.max_frame_count
                self.mt1_measurement_tracker[camera_name] = mt1_tracker[index:]

        self.io.most_recent_mt1_pose_measurements = tuple(
            [self.get_recent_mt1_pose(name) for name in self.config.camera_names]
        )
        self.io.most_recent_mt2_pose_measurements = tuple(
            [self.get_recent_mt2_pose(name) for name in self.config.camera_names]
        )

    def get_recent_mt1_pose(self, limelight_name: str) -> Pose2d:
        if limelight_name not in self.mt1_measurement_tracker:
            self.logger.warning("%s is not a valid limelight name", limelight_name)
            return Pose2d()
        if not self.mt1_measurement_tracker[limelight_name]:
            return Pose2d()

        return self.mt1_measurement_tracker[limelight_name][-1].pose

    def get_recent_mt2_pose(self, limelight_name: str) -> Pose2d:
        """
        Gets the most recent vision measurement for the given limelight name

        Args:
            limelight_name (str): Name of the limelight to measure from

        Returns:
            Pose2d: The most recent measured pose from the given limelight
        """
        if limelight_name not in self.mt2_measurement_tracker:
            self.logger.warning("%s is not a valid limelight name", limelight_name)
            return Pose2d()
        if not self.mt2_measurement_tracker[limelight_name]:
            return Pose2d()

        return self.mt2_measurement_tracker[limelight_name][-1].pose

    def get_recent_fps(self, limelight_name: str) -> float:
        """
        Gets the number of valid, processed vision measurements from the specific limelight in the last second

        Args:
            limelight_name (str): Name of the limelight to measure from

        Returns:
            int: The number of frames measured from in the last second
        """
        if limelight_name not in self.mt2_measurement_tracker:
            self.logger.warning("%s is not a valid limelight name", limelight_name)
            return 0
        return (
            len(self.mt2_measurement_tracker[limelight_name]) / self.config.time_delay
        )

    # TODO: Just a really unoptimized method so commenting it our for now. Optimize this. Just read last few measurements.
    # def has_vision(self, stale_count_time: units.seconds = 0.25) -> bool:
    #     for tracker in self.measurement_tracker.values():
    #         for measurement in reversed(tracker):
    #             if Timer.getFPGATimestamp() - measurement.timestamp < stale_count_time:
    #                 return True # More recent
    #             else:
    #                 continue # No new measurement

    #     return False

    # TODO: maybe move this into helpers
    @staticmethod
    def get_average_pose(poses: Sequence[Pose2d]):
        if not poses:
            return Pose2d()

        length = len(poses)

        if length == 1:
            return poses[0]

        x_sum = 0.0
        y_sum = 0.0

        sin_sum = 0.0
        cos_sum = 0.0

        for pose in poses:
            x_sum += pose.X()
            y_sum += pose.Y()

            sin_sum += pose.rotation().sin()
            cos_sum += pose.rotation().cos()

        avg_rot = math.atan2(sin_sum / length, cos_sum / length)
        avg_x = x_sum / length
        avg_y = y_sum / length

        return Pose2d(avg_x, avg_y, avg_rot)
