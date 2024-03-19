package frc.robot.vision

import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.vision.AprilTagCamera
import com.hamosad1657.lib.vision.RobotPoseStdDevs
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d

object AprilTagVision {
	object FrontCam : AprilTagCamera("AprilTag-Cam") {
		override val maxTagTrustingDistance = 5.meters

		override val robotToCamera
			get() = Transform3d(
				Translation3d(0.75 / 2 - 0.055, 0.75 / 2 - 0.06, 0.4),
				Rotation3d(0.0, -60.degrees.radians, 0.0)
			)

		override val stdDevs
			get() = AprilTagsStdDevs(
				oneTag = RobotPoseStdDevs(0.9, 0.9, 0.95),
				twoTagsAuto = RobotPoseStdDevs(0.5, 0.5, 0.95),
				twoTagsTeleop = RobotPoseStdDevs(0.35, 0.35, 0.95),
			)
	}
	/*
		object LeftCam : AprilTagCamera("Left-Limelight") {
			override val maxTagTrustingDistance = 4.meters

			override val robotToCamera
				get() = Transform3d(
					Translation3d(0.75 / 2 - 0.055, 0.75 / 2 - 0.06, 0.4),
					Rotation3d(0.0, -60.degrees.radians, 0.0)
				)

			override val stdDevs
				get() = AprilTagsStdDevs(
					oneTag = RobotPoseStdDevs(0.9, 0.9, 0.95),
					twoTagsAuto = RobotPoseStdDevs(0.5, 0.5, 0.95),
					twoTagsTeleop = RobotPoseStdDevs(0.35, 0.35, 0.95),
				)
		}


		object RightCam : AprilTagCamera("Right-Limelight") {
			override val maxTagTrustingDistance = 4.meters

			override val robotToCamera
				get() = Transform3d(
					Translation3d(0.75 / 2 - 0.055, 0.75 / 2 - 0.06, 0.4),
					Rotation3d(0.0, -60.degrees.radians, 0.0)
				)

			override val stdDevs
				get() = AprilTagsStdDevs(
					oneTag = RobotPoseStdDevs(0.9, 0.9, 0.95),
					twoTagsAuto = RobotPoseStdDevs(0.5, 0.5, 0.95),
					twoTagsTeleop = RobotPoseStdDevs(0.35, 0.35, 0.95),
				)
		}
		*/
}
