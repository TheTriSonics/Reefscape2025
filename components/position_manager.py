import ntcore
from magicbot import tunable

from wpimath.geometry import Pose2d, Transform2d, Rotation2d, Translation2d
from components.drivetrain import DrivetrainComponent
from utilities.position import Positions


class PositionManagerComponent:
    drivetrain: DrivetrainComponent

    reef_a_left_offset = tunable(0.0)
    reef_a_right_offset = tunable(0.0)
    reef_a_forward_offset = tunable(0.0)
    reef_a_back_offset = tunable(0.0)
    
    reef_b_left_offset = tunable(0.0)
    reef_b_right_offset = tunable(0.0)
    reef_b_forward_offset = tunable(0.0)
    reef_b_back_offset = tunable(0.0)

    reef_c_left_offset = tunable(0.0)
    reef_c_right_offset = tunable(0.0)
    reef_c_forward_offset = tunable(0.0)
    reef_c_back_offset = tunable(0.0)

    reef_d_left_offset = tunable(0.0)
    reef_d_right_offset = tunable(0.0)
    reef_d_forward_offset = tunable(0.0)
    reef_d_back_offset = tunable(0.0)
    
    reef_e_left_offset = tunable(0.0)
    reef_e_right_offset = tunable(0.0)
    reef_e_forward_offset = tunable(0.0)
    reef_e_back_offset = tunable(0.0)

    reef_f_left_offset = tunable(0.0)
    reef_f_right_offset = tunable(0.0)
    reef_f_forward_offset = tunable(0.0)
    reef_f_back_offset = tunable(0.0)

    def __init__(self):
        self.left_positions = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic("/components/position_manager/reef_left", Pose2d)
            .publish()
        )

        self.right_positions = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic("/components/position_manager/reef_right", Pose2d)
            .publish()
        )

        self.algae_positions = (
            ntcore.NetworkTableInstance.getDefault()
            .getStructArrayTopic("/components/position_manager/reef_algae", Pose2d)
            .publish()
        )

    def execute(self):
        Positions.update_alliance_positions()
        pose = self.drivetrain.get_pose()
        Positions.update_dynamic_positions(pose)

        # Now apply our offets
        Positions.REEF_A_LEFT = Positions.REEF_A_LEFT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_a_left_offset),
                Rotation2d(0),
            )
        )

        Positions.REEF_A_RIGHT = Positions.REEF_A_RIGHT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_a_right_offset),
                Rotation2d(0),
            )
        )

        Positions.REEF_B_LEFT = Positions.REEF_B_LEFT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_b_left_offset),
                Rotation2d(0),
            )
        )

        Positions.REEF_B_RIGHT = Positions.REEF_B_RIGHT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_b_right_offset),
                Rotation2d(0),
            )
        )

        Positions.REEF_C_LEFT = Positions.REEF_C_LEFT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_c_left_offset),
                Rotation2d(0),
            )
        )

        Positions.REEF_C_RIGHT = Positions.REEF_C_RIGHT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_c_right_offset),
                Rotation2d(0),
            )
        )
        Positions.REEF_D_LEFT = Positions.REEF_D_LEFT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_d_left_offset),
                Rotation2d(0),
            )
        )

        Positions.REEF_D_RIGHT = Positions.REEF_D_RIGHT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_d_right_offset),
                Rotation2d(0),
            )
        )
        Positions.REEF_E_LEFT = Positions.REEF_E_LEFT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_e_left_offset),
                Rotation2d(0),
            )
        )

        Positions.REEF_E_RIGHT = Positions.REEF_E_RIGHT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_e_right_offset),
                Rotation2d(0),
            )
        )
        Positions.REEF_F_LEFT = Positions.REEF_F_LEFT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_f_left_offset),
                Rotation2d(0),
            )
        )

        Positions.REEF_F_RIGHT = Positions.REEF_F_RIGHT.transformBy(
            Transform2d(
                Translation2d(0, -self.reef_f_right_offset),
                Rotation2d(0),
            )
        )

        lefts = [
            Positions.REEF_A_LEFT,
            Positions.REEF_B_LEFT,
            Positions.REEF_C_LEFT,
            Positions.REEF_D_LEFT,
            Positions.REEF_E_LEFT,
            Positions.REEF_F_LEFT,
        ]
        self.left_positions.set(lefts)

        rights = [
            Positions.REEF_A_RIGHT,
            Positions.REEF_B_RIGHT,
            Positions.REEF_C_RIGHT,
            Positions.REEF_D_RIGHT,
            Positions.REEF_E_RIGHT,
            Positions.REEF_F_RIGHT,
        ]
        self.right_positions.set(rights)

        centers = [
            Positions.REEF_A,
            Positions.REEF_B,
            Positions.REEF_C,
            Positions.REEF_D,
            Positions.REEF_E,
            Positions.REEF_F,
        ]
        self.algae_positions.set(centers)
