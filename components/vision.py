import time
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

    def __init__(self) -> None:
        self.timer = Timer()
        # Front cameras are backwards in left/right orientation!
        self.camera_fr = PhotonCamera("fr")
        self.camera_fl = PhotonCamera("fl")
        self.camera_fc = PhotonCamera("fc")
        self.camera_bl = PhotonCamera("bl")

        self.camera_fr_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(10.5),
                units.inchesToMeters(-12.0),
                units.inchesToMeters(7.5),
            ),
            Rotation3d.fromDegrees(0, 22.5, 14.0),
        )
        self.camera_fl_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(10.5),  # Forward/backward offset
                units.inchesToMeters(12.0),
                units.inchesToMeters(7.5),
            ),
            Rotation3d.fromDegrees(0, 22.5, -14.0),
        )

        self.camera_fc_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(-6.0),  # Forward/backward offset
                units.inchesToMeters(0.0),
                units.inchesToMeters(10.5),
            ),
            Rotation3d.fromDegrees(0.0, 0.0, 2.0),
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
            PoseStrategy.LOWEST_AMBIGUITY,
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
        linear_baseline_std = 0.10  # meters
        angular_baseline_std = math.radians(10)  # degrees to radians
        robot_pose = self.drivetrain.get_pose()
        tic: float = time.perf_counter()
        if is_sim():
            angular_baseline_std = math.radians(30)  # degrees to radians
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs
        tag_id, tag_dist = Waypoints.closest_reef_tag_id(robot_pose)
        # Check the center camera first -- If we're trusting it entirely
        # then we won't even process the others ones.
        res = self.camera_fc.getLatestResult()
        best_target = res.getBestTarget()
        if (
            best_target is not None
            and best_target.poseAmbiguity <= 0.20
            and best_target.fiducialId == tag_id
            and tag_dist < 0.75
        ):
            # This is a good one to use. Let's trust whatever this pose is
            pupdate = self.pose_estimator_fc.update(res)
            if pupdate is not None:
                twod_pose = pupdate.estimatedPose.toPose2d()
                # TODO: Publish pose to dashboard/NT
                avg_dist = best_target.getBestCameraToTarget().translation().norm()
                std_factor = (avg_dist**2)
                std_xy = linear_baseline_std * std_factor
                std_rot = angular_baseline_std * std_factor
                setDevs((std_xy, std_xy, std_rot))
                ts = self.timer.getTimestamp() - res.getLatencyMillis() / 1000.0
                self.drivetrain.estimator.addVisionMeasurement(twod_pose, ts)
                return
        toc: float = time.perf_counter()
        # print(f"Center camera took {toc-tic} seconds")

        # Use all cameras
        tic: float = time.perf_counter()
        z = zip(
            self.cameras, self.pose_estimators, self.publishers
        )

        for cam, pose_est, pub in z:
            results = cam.getAllUnreadResults()
            for res in results:
                best_target = res.getBestTarget()
                if best_target and (best_target.poseAmbiguity > 0.2):
                    # Skip using this pose in a vision update; it is too ambiguous
                    continue

                pupdate = pose_est.update(res)
                if pupdate:
                    twod_pose = pupdate.estimatedPose.toPose2d()
                    ts = self.timer.getTimestamp() - res.getLatencyMillis() / 1000.0
                    # Check if we're too far off for this to be valid
                    pub.set(twod_pose)
                    dist = robot_pose.relativeTo(twod_pose).translation().norm()
                    # Reject poses that are more than 1 meter from current
                    if dist < 1.0 or is_disabled():
                        # Ok let's figure out a stddev for this
                        total_dist = 0.0
                        tag_count = len(res.getTargets())
                        total_dist = sum(t.getBestCameraToTarget().translation().norm() for t in res.getTargets())
                        avg_dist = total_dist / tag_count
                        if avg_dist > 2.0 and not is_disabled():
                            continue  # Skip anything where the average tag is too far away
                        std_factor = (avg_dist**2) / tag_count
                        std_xy = linear_baseline_std * std_factor
                        std_rot = angular_baseline_std * std_factor
                        setDevs((std_xy, std_xy, std_rot))
                        self.drivetrain.estimator.addVisionMeasurement(twod_pose, ts)
        toc: float = time.perf_counter()
        # print(f"All cameras took {toc-tic} seconds")
