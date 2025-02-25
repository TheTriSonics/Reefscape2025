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
        self.camera_fr = PhotonCamera("fr")
        self.camera_fl = PhotonCamera("fl")

        self.camera_fr_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(11.0),
                units.inchesToMeters(-10.125),
                units.inchesToMeters(7.5),
            ),
            Rotation3d.fromDegrees(0, 27.5, 8.0),
        )
        self.camera_fl_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(11.0), # Forward/backward offset
                units.inchesToMeters(10.125),
                units.inchesToMeters(7.5),
            ),
            Rotation3d.fromDegrees(0, 27.5, -8.0),
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

        self.cameras = [self.camera_fr, self.camera_fl]
        self.pose_estimators = [self.pose_estimator_fr, self.pose_estimator_fl]
        self.publishers = [self.publisher_fr, self.publisher_fl]

    def execute(self) -> None:
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs
        tag_id, tag_dist = Waypoints.closest_reef_tag_id(self.drivetrain.get_pose())
        for cam, pose_est, pub in zip(
            self.cameras, self.pose_estimators, self.publishers
        ):
            results = cam.getAllUnreadResults()
            for res in results:
                best_target = res.getBestTarget()
                if best_target and best_target.poseAmbiguity > 0.2:
                    # Skip using this pose in a vision update; it is too ambiguous
                    continue

                taget_ids_in_frame = [t.fiducialId for t in res.getTargets()]
                if tag_dist < 2.0 and tag_id in taget_ids_in_frame:
                    # We're close to an apriltag, so we should use that for
                    # vision. We'll tighten up the std devs to make sure we
                    # are trusting this reading.
                    self.std_x, self.std_y, self.std_rot = 0.1, 0.1, 0.2
                else:
                    self.std_x, self.std_y, self.std_rot = 0.4, 0.4, 0.2

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
