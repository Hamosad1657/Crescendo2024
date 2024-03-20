package com.hamosad1657.lib.vision

import com.hamosad1657.lib.Alert
import com.hamosad1657.lib.Alert.AlertType.ERROR
import com.hamosad1657.lib.units.Length
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

private val TAGS_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()

abstract class AprilTagCamera(val cameraName: String) {
	protected abstract val robotToCamera: Transform3d

	abstract val maxTagTrustingDistance: Length
	abstract val stdDevs: AprilTagsStdDevs

	private var _camera: PhotonCamera? = null
	protected val camera: PhotonCamera
		get() = _camera ?: PhotonCamera(cameraName).also { _camera = it }


	// Try to retrieve the pose estimator again only if it is null (it will be null if the camera is)
	private var _poseEstimator: PhotonPoseEstimator? = null
	protected val poseEstimator: PhotonPoseEstimator
		get() =
			_poseEstimator ?: PhotonPoseEstimator(
				TAGS_LAYOUT,
				MULTI_TAG_PNP_ON_COPROCESSOR,
				camera,
				robotToCamera,
			).apply {
				setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
			}.also {
				_poseEstimator = it
			}

	private val disconnectedAlert = Alert("$cameraName disconnected", ERROR)

	val isConnected
		get() = camera.isConnected
			.also { disconnectedAlert.set(!it) }

	val isInRange: Boolean
		get() {
			val robotToTagDistance = bestTag?.bestCameraToTarget?.x ?: return false
			return robotToTagDistance < maxTagTrustingDistance.asMeters
		}

	val latestResult: PhotonPipelineResult? get() = if (isConnected) camera.latestResult else null
	val bestTag: PhotonTrackedTarget? get() = latestResult?.bestTarget

	fun getTag(tagID: Int) = latestResult?.targets?.find { it.fiducialId == tagID }
	fun isTagDetected(tagID: Int) = getTag(tagID) != null
	fun isAnyTagDetected(vararg tagIDs: Int) = tagIDs.any(::isTagDetected)

	/**
	 * Gets the estimated robot position from the PhotonVision camera.
	 *
	 * Returns null if it doesn't detect any AprilTags.
	 */
	val estimatedGlobalPose: EstimatedRobotPose?
		get() = if (isConnected) poseEstimator.update()?.orElse(null) else null

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
