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

	private val robotToCamera =
		Transform3d(
			Translation3d(0.135, 0.375, -0.465),
			Rotation3d(0.degrees.radians, 60.degrees.radians, 90.degrees.radians)
		)
	private val camera = PhotonCamera("AprilTag-Cam")

	var aprilTags: AprilTagFieldLayout =
		AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()

	private val poseEstimator =
		PhotonPoseEstimator(
			aprilTags,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			camera,
			robotToCamera,
		).apply {
			setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
		}

	/**
	 * Gets the estimated robot position from the PhotonVision camera.
	 * Returns null if it doesn't detect any April Tags.
	 */
	val estimatedGlobalPose: EstimatedRobotPose? get() = poseEstimator.update().orElse(null)

	val estimatedPose2d: Pose2d? get() = estimatedGlobalPose?.estimatedPose?.toPose2d()

	val latestResult: PhotonPipelineResult get() = camera.latestResult

	val bestTag: PhotonTrackedTarget get() = latestResult.bestTarget

	fun getTag(tagID: Int) = latestResult.getTargets().getOrNull(tagID)
}
