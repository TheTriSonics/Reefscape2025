import math
import wpilib
from magicbot import tunable, will_reset_to
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

    tracking_tag = will_reset_to(False)

    def __init__(self) -> None:
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
        linear_baseline_std = 0.1  # meters
        angular_baseline_std = math.radians(10)  # degrees to radians
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs
        tag_id, tag_dist = Waypoints.closest_reef_tag_id(self.drivetrain.get_pose())

        # Use all cameras
        z = zip(
            self.cameras, self.pose_estimators, self.publishers
        )

        for cam, pose_est, pub in z:
            results = cam.getAllUnreadResults()
            for res in results:
                best_target = res.getBestTarget()
                if best_target and (best_target.poseAmbiguity > 0.2) and not is_sim():
                    print("Skipping pose due to high ambiguity")
                    # Skip using this pose in a vision update; it is too ambiguous
                    continue

                res_time = res.getTimestampSeconds()
                pupdate = pose_est.update(res)
                if pupdate:
                    twod_pose = pupdate.estimatedPose.toPose2d()
                    pub.set(twod_pose)
                    # Check if we're too far off for this to be valid
                    robot_pose = self.drivetrain.get_pose()
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
                        if self.tracking_tag:
                            if best_target and best_target.fiducialId != tag_id:
                                std_factor *= 5.0
                                print(f"Increasing vision_std due to tag mismatch where {best_target.fiducialId} != {tag_id} and {cam.getName()}")
                            elif best_target and best_target.fiducialId == tag_id:
                                print(f"Decreasing vision_std due to tag match where {best_target.fiducialId} == {tag_id} and {cam.getName()}")
                                std_factor *= 0.1
                        std_xy = linear_baseline_std * std_factor
                        # only print results every 3 seconds
                        # if Timer.getFPGATimestamp() % 3 < 0.1:
                        #     print(f"camera is {cam.getName()} and target is {best_target.fiducialId} and tracking target is {self.tracking_tag} and {std_xy=}")
                        std_rot = angular_baseline_std * std_factor
                        setDevs((std_xy, std_xy, std_rot))
                        self.drivetrain.estimator.addVisionMeasurement(twod_pose, res_time)
