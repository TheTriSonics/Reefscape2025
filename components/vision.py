import math
import wpilib
from magicbot import tunable
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import Timer
import ntcore
from wpimath.geometry import Pose2d, Pose3d, Translation2d, Translation3d, Rotation3d, Rotation2d, Transform3d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from components.drivetrain import DrivetrainComponent
from wpimath import units
from utilities.game import is_sim, is_disabled
from utilities import Waypoints


class VisionComponent():

    drivetrain: DrivetrainComponent

    all_cams = tunable(False)
    center_only = tunable(False)

    def __init__(self) -> None:
        self.timer = Timer()
        # Front cameras are backwards in left/right orientation!
        self.camera_fr = PhotonCamera("fl")
        self.camera_fl = PhotonCamera("fr")
        self.camera_fc = PhotonCamera("fc")
        self.camera_bl = PhotonCamera("bl")

        self.camera_fr_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(10.5),
                units.inchesToMeters(-12.0),
                units.inchesToMeters(7.5),
            ),
            Rotation3d.fromDegrees(0, 22.5, 8.0),
        )
        self.camera_fl_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(10.5),  # Forward/backward offset
                units.inchesToMeters(12.0),
                units.inchesToMeters(7.5),
            ),
            Rotation3d.fromDegrees(0, 22.5, -8.0),
        )

        self.camera_fc_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(5.5),  # Forward/backward offset
                units.inchesToMeters(0.0),
                units.inchesToMeters(12.5),
            ),
            Rotation3d.fromDegrees(0, 22.5, 0.0),
        )

        self.camera_bl_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(-13.0),  # Forward/backward offset
                units.inchesToMeters(10.125),
                units.inchesToMeters(7.5),
            ),
            Rotation3d.fromDegrees(0, -22.5, 188.0),
        )

        field = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)

        self.pose_estimator_fr = PhotonPoseEstimator(
            field,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera_fr,
            self.camera_fr_offset,
        )
        self.pose_estimator_fl = PhotonPoseEstimator(
            field,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera_fl,
            self.camera_fl_offset,
        )
        self.pose_estimator_fc = PhotonPoseEstimator(
            field,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera_fc,
            self.camera_fc_offset,
        )
        self.pose_estimator_bl = PhotonPoseEstimator(
            field,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera_bl,
            self.camera_bl_offset,
        )

        self.publisher_fr = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_fr", Pose2d)
            .publish()
        )
        self.publisher_fl = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_fl", Pose2d)
            .publish()
        )
        self.publisher_fc = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_fc", Pose2d)
            .publish()
        )
        self.publisher_bl = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_bl", Pose2d)
            .publish()
        )

        self.cameras = [self.camera_fr, self.camera_fl, self.camera_fc, self.camera_bl]
        self.pose_estimators = [
            self.pose_estimator_fr,
            self.pose_estimator_fl,
            self.pose_estimator_fc,
            self.pose_estimator_bl,
        ]
        self.publishers = [
            self.publisher_fr,
            self.publisher_fl,
            self.publisher_fc,
            self.publisher_bl,
        ]

    def execute(self) -> None:
        from math import radians
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs
        tag_id, tag_dist = Waypoints.closest_reef_tag_id(self.drivetrain.get_pose())
        if tag_dist < 1.0:
            # Only use the center camera
            z = zip([self.camera_fc], [self.pose_estimator_fc], [self.publisher_fc])
            self.center_only = True
            self.all_cams = False
        else:
            # Use all cameras
            z = zip(
                self.cameras, self.pose_estimators, self.publishers
            )
            self.center_only = False
            self.all_cams = True

        for cam, pose_est, pub in z:
            results = cam.getAllUnreadResults()
            for res in results:
                best_target = res.getBestTarget()
                if best_target and (best_target.poseAmbiguity > 0.3):
                    # Skip using this pose in a vision update; it is too ambiguous
                    # continue
                    pass
                linear_baseline_std = 0.02  # meters
                angular_baseline_std = 0.06  # radians

                pupdate = pose_est.update(res)
                if pupdate:
                    twod_pose = pupdate.estimatedPose.toPose2d()
                    ts = self.timer.getTimestamp() - res.getLatencyMillis() / 1000.0
                    # Check if we're too far off for this to be valid
                    robot_pose = self.drivetrain.get_pose()
                    pub.set(twod_pose)
                    dist = robot_pose.relativeTo(twod_pose).translation().norm()
                    # Reject poses that are more than 1 meter from current
                    if dist < 1.0 or is_disabled():
                        # Ok let's figure out a stddev for this
                        total_dist = 0.0
                        tag_count = len(res.getTargets())
                        for t in res.getTargets():
                            total_dist += t.getBestCameraToTarget().translation().norm()
                        avg_dist = total_dist / tag_count
                        std_factor = (avg_dist**2) / tag_count
                        std_xy = linear_baseline_std * std_factor
                        std_rot = angular_baseline_std * std_factor
                        setDevs((std_xy, std_xy, std_rot))
                        self.drivetrain.estimator.addVisionMeasurement(twod_pose, ts)
