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

    std_x = tunable(0.4)
    std_y = tunable(0.4)
    std_rot = tunable(0.2)

    def __init__(self) -> None:
        self.timer = Timer()
        # Front cameras are backwards in left/right orientation!
        self.camera_fr = PhotonCamera("fl")
        self.camera_fl = PhotonCamera("fr")
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
        self.publisher_bl = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("/components/vision/pose_bl", Pose2d)
            .publish()
        )

        self.cameras = [self.camera_fr, self.camera_fl, self.camera_bl]
        self.pose_estimators = [
            self.pose_estimator_fr,
            self.pose_estimator_fl,
            self.pose_estimator_bl,
        ]
        self.publishers = [self.publisher_fr, self.publisher_fl, self.publisher_bl]

    def execute(self) -> None:
        from math import radians
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs
        tag_id, tag_dist = Waypoints.closest_reef_tag_id(self.drivetrain.get_pose())
        for cam, pose_est, pub in zip(
            self.cameras, self.pose_estimators, self.publishers
        ):
            results = cam.getAllUnreadResults()
            for res in results:
                best_target = res.getBestTarget()
                if best_target and (best_target.poseAmbiguity > 0.2):
                    # Skip using this pose in a vision update; it is too ambiguous
                    continue

                taget_ids_in_frame = [t.fiducialId for t in res.getTargets()]
                if tag_dist < 0.5 and tag_id in taget_ids_in_frame:
                    self.std_x, self.std_y, self.std_rot = 0.01, 0.01, radians(5.0)
                elif tag_dist < 1.0 and tag_id in taget_ids_in_frame:
                    self.std_x, self.std_y, self.std_rot = 0.05, 0.05, radians(10)
                elif tag_dist < 2.0 and tag_id in taget_ids_in_frame:
                    self.std_x, self.std_y, self.std_rot = 0.1, 0.1, radians(22.5)
                else:
                    self.std_x, self.std_y, self.std_rot = 0.4, 0.4, radians(22.5)

                setDevs((self.std_x, self.std_y, self.std_rot))
                pupdate = pose_est.update(res)
                if pupdate:
                    pose = pupdate.estimatedPose
                    twod_pose = Pose2d(pose.x, pose.y,
                                        pose.rotation().toRotation2d())
                    pub.set(twod_pose)
                    ts = self.timer.getTimestamp() - res.getLatencyMillis() / 1000.0
                    # Check if we're too far off for this to be valid
                    robot_pose = self.drivetrain.get_pose()
                    xdiff = abs(robot_pose.X() -twod_pose.X())
                    ydiff = abs(robot_pose.Y() -twod_pose.Y())
                    distance = math.sqrt(xdiff**2 + ydiff**2) 
                    if distance < 1 or is_disabled():
                        self.drivetrain.estimator.addVisionMeasurement(twod_pose, ts)
