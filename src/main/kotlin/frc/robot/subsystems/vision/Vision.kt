package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.*
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.PI

object Vision : SubsystemBase() {
	private val robotToCamera = Transform3d(Translation3d(), Rotation3d(0.0, 0.0, PI / 2))
	private val camera = PhotonCamera("AprilTag-Cam")

	var aprilTags: AprilTagFieldLayout =
		AprilTagFieldLayout(
			listOf(
				AprilTag(
					5,
					Pose3d(
						0.0, 0.0, 0.0,
						Rotation3d(0.0, 0.0, 0.0)
					)
				)
			), 10.0, 10.0
		)

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
	var estimatedGlobalPose: EstimatedRobotPose? = poseEstimator.update().orElse(null)

	val latestResult: PhotonPipelineResult get() = camera.latestResult

	val bestTag: PhotonTrackedTarget get() = latestResult.bestTarget

	fun calibrateTagPositions(transform: Transform3d) {
		aprilTags = AprilTagFieldLayout(
			latestResult.targets.map { target ->
				AprilTag(
					target.fiducialId,
					(target.bestCameraToTarget + transform).let { Pose3d(it.translation, it.rotation) }
				)
			}, 10.0, 10.0
		)
	}

	fun getTag(tagID: Int) = latestResult.getTargets().getOrNull(tagID)

	override fun periodic() {
		poseEstimator.update().orElse(null)
	}
}
