package frc.robot.subsystems.vision

import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.meters
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import org.photonvision.*
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

object Vision {
	val MAX_VISION_TO_ODOMETRY_DELTA = 1.0.meters

	private val camera: PhotonCamera? = try {
		PhotonCamera("AprilTag-Cam")
	} catch (_: Exception) {
		null
	}

	private val robotToCamera =
		Transform3d(
			Translation3d(0.135, 0.375, -0.465),
			Rotation3d(0.degrees.radians, 60.degrees.radians, 90.degrees.radians)
		)

	var aprilTags: AprilTagFieldLayout =
		AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()

	private val poseEstimator: PhotonPoseEstimator? = if (camera != null) {
		PhotonPoseEstimator(
			aprilTags,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			camera,
			robotToCamera,
		).apply {
			setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
		}
	} else {
		null
	}

	/**
	 * Gets the estimated robot position from the PhotonVision camera.
	 * Returns null if it doesn't detect any AprilTags.
	 */
	val estimatedGlobalPose: EstimatedRobotPose? get() = poseEstimator?.update()?.orElse(null)

	val estimatedPose2d: Pose2d? get() = estimatedGlobalPose?.estimatedPose?.toPose2d()

	val latestResult: PhotonPipelineResult? get() = camera?.latestResult

	val bestTag: PhotonTrackedTarget? get() = latestResult?.bestTarget

	fun getTag(tagID: Int): PhotonTrackedTarget? = latestResult?.getTargets()?.getOrNull(tagID)
}
