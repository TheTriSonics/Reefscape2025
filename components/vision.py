import math
import wpilib
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import Timer
import ntcore
from wpimath.geometry import Pose2d, Pose3d, Translation2d, Translation3d, Rotation3d, Rotation2d, Transform3d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from components.drivetrain import DrivetrainComponent
from wpimath import units
from utilities.game import is_sim, is_disabled



class VisionComponent():

    drivetrain: DrivetrainComponent


    def __init__(self) -> None:
        self.timer = Timer()
        self.camera_center = PhotonCamera("ardu_cam-1")
        self.camera_fl = PhotonCamera("ardu_cam-2")

        self.camera_center_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(1.5),
                units.inchesToMeters(0.5),
                units.inchesToMeters(16),
            ),
            Rotation3d.fromDegrees(0, 0, 0),
        )
        self.camera_fl_offset = Transform3d(
            Translation3d(
                units.inchesToMeters(16.75), # Forward/backward offset
                units.inchesToMeters(10.25),
                units.inchesToMeters(5.75),
            ),
            Rotation3d.fromDegrees(0, 0, 0),
        )

        field = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)
        wpilib.SmartDashboard.putNumber('field length (m)', field.getFieldLength())
        wpilib.SmartDashboard.putNumber('field width (m)', field.getFieldWidth())

        self.pose_estimator_center = PhotonPoseEstimator(
            field,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera_center,
            self.camera_center_offset,
        )
        self.pose_estimator_fl = PhotonPoseEstimator(
            field,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera_fl,
            self.camera_fl_offset,
        )

        self.publisher_center = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("PhotonPose_center", Pose2d)
            .publish()
        )
        self.publisher_fl = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructTopic("PhotonPose_fl", Pose2d)
            .publish()
        )

        self.cameras = [self.camera_center, self.camera_fl]
        self.pose_estimators = [self.pose_estimator_center, self.pose_estimator_fl]
        self.publishers = [self.publisher_center, self.publisher_fl]

    def execute(self) -> None:
        if is_sim():
            # Skip vision on sim for now
            return
        setDevs = self.drivetrain.estimator.setVisionMeasurementStdDevs
        for cam, pose_est, pub in zip(
            self.cameras, self.pose_estimators, self.publishers
        ):
            results = cam.getAllUnreadResults()
            for res in results:
                best_target = res.getBestTarget()
                if best_target and best_target.poseAmbiguity > 0.2:
                    # Skip using this pose in a vision update; it is too ambiguous
                    continue
                pupdate = pose_est.update(res)
                if pupdate:
                    pose = pupdate.estimatedPose
                    twod_pose = Pose2d(pose.x, pose.y,
                                        pose.rotation().toRotation2d())
                    pub.set(twod_pose)
                    ts = self.timer.getTimestamp() - res.getLatencyMillis() / 1000.0
                    tv, rv = self.drivetrain.get_robot_speeds()
                    # TODO: Take into account rv, rotational velocity
                    # for standard deviations. A spinning robot is not accurate!
                    """
                    # Getting rid of this dynamic std deviation idea.
                    # I think this is a hold-over from days where the
                    # camera was mounted higher on the robot and it would
                    # vibrate when it ran
                    std_x = (0.4 * max(abs(tv**1.5), 1)) / target_count
                    std_y = std_x
                    std_rot = std_x / 2
                    """
                    std_x, std_y, std_rot = 0.4, 0.4, 0.2
                    setDevs((std_x, std_y, std_rot))
                    # Check if we're too far off for this to be valid
                    robot_pose = self.drivetrain.get_pose()
                    xdiff = abs(robot_pose.X() -twod_pose.X())
                    ydiff = abs(robot_pose.Y() -twod_pose.Y())
                    distance = math.sqrt(xdiff**2 + ydiff**2) 
                    if distance < 1 or is_disabled():
                        self.drivetrain.estimator.addVisionMeasurement(twod_pose, ts)
