from wpilib import Timer
import ntcore
from wpimath.geometry import Pose2d, Pose3d, Translation2d, Translation3d, Rotation3d, Rotation2d
from photonlibpy.photonCamera import PhotonCamera
from components.drivetrain import DrivetrainComponent
from wpimath import units

class Vision():

    chassis: DrivetrainComponent

    def __init__(self) -> None:
        self.timer = Timer()
        self.camera = PhotonCamera("ardu_cam-1")
        self.publisher = (ntcore.NetworkTableInstance.getDefault()
                                                .getStructTopic("PhotonPose", Pose2d)
                                                .publish()
        )
        self.camera_offset = Pose3d(units.inchesToMeters(1.5),
                                    units.inchesToMeters(0.5),
                                    units.inchesToMeters(16),
                                    Rotation3d(0, 0, 0),
                        )

    def execute(self) -> None:
        results = self.camera.getAllUnreadResults()
        for res in results:
            if res.multitagResult:
                p = res.multitagResult.estimatedPose.best
                t = Translation3d(p.x, p.y, p.z)
                r = p.rotation()
                vision_pose = Pose3d(t, r)
                tmp = vision_pose - self.camera_offset
                vision_pose = Pose3d(tmp.translation(), tmp.rotation())
                twod_pose = Pose2d(vision_pose.x,
                                   vision_pose.y,
                                   vision_pose.rotation().toRotation2d())
                self.publisher.set(twod_pose)
                """
                # TODO: We can come up with a dynamic way of determining these
                xdev, ydev, rdev = 0.5, 0.5, 0.2
                self.chassis.estimator.setVisionMeasurementStdDevs((xdev, ydev, rdev))
                """
                self.chassis.estimator.addVisionMeasurement(twod_pose,
                                                            self.timer.getFPGATimestamp())
