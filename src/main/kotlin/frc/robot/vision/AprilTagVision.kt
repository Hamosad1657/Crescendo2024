package frc.robot.vision

import com.hamosad1657.lib.Alert
import com.hamosad1657.lib.Alert.AlertType.ERROR
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
	private val disconnectedAlert = Alert("AprilTag-Cam disconnected", ERROR)

	private val camera: PhotonCamera? = try {
		PhotonCamera("AprilTag-Cam")
	} catch (_: Exception) {
		null
	}

	private val robotToCamera =
		Transform3d(
			Translation3d((0.75 / 2 - 0.055), 0.75 / 2 - 0.06, 0.4),
			Rotation3d(0.0, -60.degrees.radians, 0.0)
		)

	private var aprilTags: AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()

	private val poseEstimator: PhotonPoseEstimator? =
		camera?.let {
			PhotonPoseEstimator(
				aprilTags,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				it,
				robotToCamera,
			).apply {
				setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
			}
		}

	val isConnected: Boolean get() = (camera?.isConnected ?: false).also {
		disconnectedAlert.set(!it)
	}

	val poseEstimationStdDevs
		get() = if ((latestResult?.targets?.size == 1)) {
			SEEING_ONE_TAG_STANDARD_DEVS
		} else if (frc.robot.Robot.isAutonomous) {
			AUTO_SEEING_TWO_TAGS_STANDARD_DEVS
		} else {
			TELEOP_SEEING_TWO_TAGS_STANDARD_DEVS
		}

	/**
	 * Gets the estimated robot position from the PhotonVision camera.
	 *
	 * Returns null if it doesn't detect any AprilTags.
	 */
	val estimatedGlobalPose: EstimatedRobotPose? get() = if (isConnected) poseEstimator?.update()?.orElse(null) else null
	val estimatedPose2d: Pose2d? get() = if (isConnected) estimatedGlobalPose?.estimatedPose?.toPose2d() else null

	val latestResult: PhotonPipelineResult? get() = if (isConnected) camera?.latestResult else null

	val bestTag: PhotonTrackedTarget? get() = if (isConnected) latestResult?.bestTarget else null
	fun getTag(tagID: Int): PhotonTrackedTarget? = if (isConnected) latestResult?.getTargets()?.find { it.fiducialId == tagID } else null


	val MAX_TAG_TRUSTING_DISTANCE = 5.0.meters

	val SEEING_ONE_TAG_STANDARD_DEVS = Matrix(Nat.N3(), Nat.N1()).apply {
		this[0, 0] = 0.9 // Translation X
		this[1, 0] = 0.9 // Translation Y
		this[2, 0] = 0.95 // Rotation
	}

	val AUTO_SEEING_TWO_TAGS_STANDARD_DEVS = Matrix(Nat.N3(), Nat.N1()).apply {
		this[0, 0] = 0.5 // Translation X
		this[1, 0] = 0.5 // Translation Y
		this[2, 0] = 0.95 // Rotation
	}

	val TELEOP_SEEING_TWO_TAGS_STANDARD_DEVS = Matrix(Nat.N3(), Nat.N1()).apply {
		this[0, 0] = 0.35 // Translation X
		this[1, 0] = 0.35 // Translation Y
		this[2, 0] = 0.95 // Rotation
	}
}
