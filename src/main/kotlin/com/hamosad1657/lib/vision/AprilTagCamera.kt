package com.hamosad1657.lib.vision

import com.hamosad1657.lib.units.meters
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import frc.robot.Robot
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

class RobotPoseStdDevs(
	translationX: Double,
	translationY: Double,
	rotation: Double,
) : Matrix<N3, N1>(Nat.N3(), Nat.N1()) {
	init {
		this[0, 0] = translationX
		this[1, 0] = translationY
		this[2, 0] = rotation
	}
}

private val TAGS_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()

abstract class AprilTagCamera(cameraName: String) {
	protected val camera: PhotonCamera? =
		try {
			PhotonCamera(cameraName)
		} catch (_: Exception) {
			null
		}

	protected abstract val robotToCamera: Transform3d

	protected val poseEstimator: PhotonPoseEstimator? =
		camera?.let {
			PhotonPoseEstimator(
				TAGS_LAYOUT,
				MULTI_TAG_PNP_ON_COPROCESSOR,
				camera,
				robotToCamera,
			).apply {
				setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
			}
		}

	/**
	 * Gets the estimated robot position from the PhotonVision camera.
	 *
	 * Returns null if it doesn't detect any AprilTags.
	 */
	val estimatedGlobalPose: EstimatedRobotPose? get() = poseEstimator?.update()?.orElse(null)

	val latestResult: PhotonPipelineResult? get() = camera?.latestResult
	val bestTag: PhotonTrackedTarget? get() = latestResult?.bestTarget

	open val maxTagTrustingDistance = 5.meters
	abstract val stdDevs: AprilTagsStdDevs

	val poseEstimationStdDevs
		get() = if (latestResult?.targets?.size == 1) {
			stdDevs.oneTag
		} else if (Robot.isAutonomous) {
			stdDevs.twoTagsAuto
		} else {
			stdDevs.twoTagsTeleop
		}

	data class AprilTagsStdDevs(
		val oneTag: RobotPoseStdDevs,
		val twoTagsAuto: RobotPoseStdDevs,
		val twoTagsTeleop: RobotPoseStdDevs,
	)
}