import ntcore
from wpimath.geometry import Pose2d, Translation2d
from photonlibpy.photonCamera import PhotonCamera
from components.drivetrain import DrivetrainComponent


class Vision():

    chassis: DrivetrainComponent

    def __init__(self) -> None:
        self.camera = PhotonCamera("ardu_cam-1")
        self.publisher = (ntcore.NetworkTableInstance.getDefault()
                                                .getStructTopic("PhotonPose", Pose2d)
                                                .publish()
        )

    def execute(self) -> None:
        results = self.camera.getAllUnreadResults()
        for res in results:
            if res.multitagResult:
                print('setting pose from vision')
                p = res.multitagResult.estimatedPose
                t = Translation2d(p.best.x, p.best.y)
                r = p.best.rotation().toRotation2d()
                vision_pose = Pose2d(t, r)
                self.publisher.set(vision_pose)
                xdev, ydev, rdev = 0.5, 0.5, 0.2 
                self.chassis.estimator.setVisionMeasurementStdDevs((xdev, ydev, rdev))
                self.chassis.estimator.addVisionMeasurement(vision_pose, 0)
