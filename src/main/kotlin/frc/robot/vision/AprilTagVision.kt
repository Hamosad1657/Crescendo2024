package frc.robot.vision

import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.vision.AprilTagCamera
import com.hamosad1657.lib.vision.RobotPoseStdDevs
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d

object AprilTagVision {
	object RPi4 : AprilTagCamera("AprilTag-Cam") {
		override val robotToCamera =
			Transform3d(
				Translation3d((0.75 / 2 - 0.055), 0.75 / 2 - 0.06, 0.4),
				Rotation3d(0.0, -60.degrees.radians, 0.0)
			)

		override val maxTagTrustingDistance = 5.meters
		override val stdDevs = AprilTagsStdDevs(
			oneTag = RobotPoseStdDevs(
				translationX = 0.9,
				translationY = 0.9,
				rotation = 0.95,
			),
			twoTagsAuto = RobotPoseStdDevs(
				translationX = 0.5,
				translationY = 0.5,
				rotation = 0.95,
			),
			twoTagsTeleop = RobotPoseStdDevs(
				translationX = 0.35,
				translationY = 0.35,
				rotation = 0.95,
			),
		)
	}

	object LeftLimelight : AprilTagCamera("Left-Limelight") {
		override val robotToCamera =
			Transform3d(
				Translation3d((0.75 / 2 - 0.055), 0.75 / 2 - 0.06, 0.4),
				Rotation3d(0.0, -60.degrees.radians, 0.0)
			)

		override val maxTagTrustingDistance = 5.meters
		override val stdDevs = AprilTagsStdDevs(
			oneTag = RobotPoseStdDevs(
				translationX = 0.9,
				translationY = 0.9,
				rotation = 0.95,
			),
			twoTagsAuto = RobotPoseStdDevs(
				translationX = 0.5,
				translationY = 0.5,
				rotation = 0.95,
			),
			twoTagsTeleop = RobotPoseStdDevs(
				translationX = 0.35,
				translationY = 0.35,
				rotation = 0.95,
			),
		)
	}

	object RightLimelight : AprilTagCamera("Right-Limelight") {
		override val robotToCamera =
			Transform3d(
				Translation3d((0.75 / 2 - 0.055), 0.75 / 2 - 0.06, 0.4),
				Rotation3d(0.0, -60.degrees.radians, 0.0)
			)

		override val maxTagTrustingDistance = 5.meters
		override val stdDevs = AprilTagsStdDevs(
			oneTag = RobotPoseStdDevs(
				translationX = 0.9,
				translationY = 0.9,
				rotation = 0.95,
			),
			twoTagsAuto = RobotPoseStdDevs(
				translationX = 0.5,
				translationY = 0.5,
				rotation = 0.95,
			),
			twoTagsTeleop = RobotPoseStdDevs(
				translationX = 0.35,
				translationY = 0.35,
				rotation = 0.95,
			),
		)
	}
}
