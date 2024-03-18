package frc.robot.vision

import com.hamosad1657.lib.Alert
import com.hamosad1657.lib.Alert.AlertType.ERROR
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.vision.AprilTagCamera
import com.hamosad1657.lib.vision.RobotPoseStdDevs
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

object AprilTagVision {
	object FrontCam: AprilTagCamera("AprilTag-Cam") {

		val MAX_TAG_TRUSTING_DISTANCE = 5.0.meters

		val isInRange: Boolean get() {
			val robotToTagDistance = AprilTagVision.FrontCam.bestTag?.bestCameraToTarget?.x ?: return false
			return robotToTagDistance < AprilTagVision.FrontCam.MAX_TAG_TRUSTING_DISTANCE.asMeters
		}

		override val robotToCamera: Transform3d
			get() = Transform3d(
				Translation3d((0.75 / 2 - 0.055), 0.75 / 2 - 0.06, 0.4),
				Rotation3d(0.0, -60.degrees.radians, 0.0)
			)

		override val stdDevs: AprilTagsStdDevs
			get() = AprilTagsStdDevs(
				RobotPoseStdDevs(0.9, 0.9, 0.95),
				RobotPoseStdDevs(0.5, 0.5, 0.95),
				RobotPoseStdDevs(0.35, 0.35, 0.95)
			)
	}
}
