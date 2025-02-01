import wpilib
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import Timer
import ntcore
from wpimath.geometry import Pose2d, Pose3d, Translation2d, Translation3d, Rotation3d, Rotation2d, Transform3d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from components.drivetrain import DrivetrainComponent
from wpimath import units


class Vision():

    chassis: DrivetrainComponent

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
                units.inchesToMeters(10.25),
                units.inchesToMeters(10.25),
                units.inchesToMeters(5.75),
            ),
            Rotation3d.fromDegrees(0, 27, 45),
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
        """
        # Disabled so we can run only one camera for now
        self.cameras = [self.camera_center, self.camera_fl]
        self.pose_estimators = [self.pose_estimator_center, self.pose_estimator_fl]
        self.publishers = [self.publisher_center, self.publisher_fl]
        """

        self.cameras = [self.camera_center]
        self.pose_estimators = [self.pose_estimator_center]
        self.publishers = [self.publisher_center]

    def execute(self) -> None:
        setDevs = self.chassis.estimator.setVisionMeasurementStdDevs
        for cam, pose_est, pub in zip(
            self.cameras, self.pose_estimators, self.publishers
        ):
            results = cam.getAllUnreadResults()
            for res in results:
                target_count = len(res.getTargets())
                pupdate = pose_est.update(res)
                if pupdate:
                    pose = pupdate.estimatedPose
                    twod_pose = Pose2d(pose.x, pose.y,
                                        pose.rotation().toRotation2d())
                    pub.set(twod_pose)
                    ts = self.timer.getTimestamp() - res.getLatencyMillis() / 1000.0
                    tv, rv = self.chassis.get_robot_speeds()
                    # TODO: Take into account rv, rotational velocity
                    # for standard deviations. A spinning robot is not accurate!
                    std_x = (0.4 * max(abs(tv**1.5), 1)) / target_count
                    std_y = std_x
                    std_rot = std_x / 2
                    setDevs((std_x, std_y, std_rot))
                    self.chassis.estimator.addVisionMeasurement(twod_pose, ts)
