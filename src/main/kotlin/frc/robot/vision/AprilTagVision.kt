package frc.robot.vision

import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.meters
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
	val MAX_VISION_TO_ODOMETRY_DELTA = 1.0.meters

	private val camera: PhotonCamera? = try {
		PhotonCamera("AprilTag-Cam")
	} catch (_: Exception) {
		null
	}

	private val robotToCamera =
		Transform3d(
			Translation3d((0.75 / 2 - 0.055), 0.75 / 2 - 0.06, -0.4),
			Rotation3d(0.0, 60.degrees.radians, 0.0)
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

	val poseEstimationStdDevs
		get() = Matrix(Nat.N3(), Nat.N1()).apply {
			if (latestResult!!.targets.size == 1) {
				this[0, 0] = 0.9 // Translation X
				this[1, 0] = 0.9 // Translation Y
				this[2, 0] = 0.95 // Rotation
			}
			this[0, 0] = 0.3 // Translation X
			this[1, 0] = 0.3 // Translation Y
			this[2, 0] = 0.95 // Rotation
		}

	/**
	 * Gets the estimated robot position from the PhotonVision camera.
	 *
	 * Returns null if it doesn't detect any AprilTags.
	 */
	val estimatedGlobalPose: EstimatedRobotPose? get() = poseEstimator?.update()?.orElse(null)
	val estimatedPose2d: Pose2d? get() = estimatedGlobalPose?.estimatedPose?.toPose2d()

	val latestResult: PhotonPipelineResult? get() = camera?.latestResult

	val bestTag: PhotonTrackedTarget? get() = latestResult?.bestTarget
	fun getTag(tagID: Int): PhotonTrackedTarget? = latestResult?.getTargets()?.find { it.fiducialId == tagID }
}
