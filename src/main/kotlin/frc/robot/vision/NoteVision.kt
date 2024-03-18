package frc.robot.vision

import com.hamosad1657.lib.Alert
import com.hamosad1657.lib.Alert.AlertType.ERROR
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.minus
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

object NoteVision : Sendable {

	private val disconnectedAlert = Alert("NOTE-Cam disconnected", ERROR)

	/**
	 * The angle to the NOTE as reported by the vision,
	 * when the intake is perfectly centered at it.
	 */
	private val CAMERA_OFFSET = 0.0.degrees

	private val camera: PhotonCamera? = try {
		PhotonCamera("NOTE-Cam")
	} catch (_: Exception) {
		null
	}

	val isConnected: Boolean get() = (camera?.isConnected ?: false).also {
		disconnectedAlert.set(!it)
	}

	val latestResult: PhotonPipelineResult? get() = if (isConnected) camera?.latestResult else null
	val bestTarget: PhotonTrackedTarget? get() = latestResult?.bestTarget
	val hasTargets: Boolean get() = latestResult?.hasTargets() ?: false

	/** Get the yaw delta between the robot and the target note. */
	private fun getRobotToTargetYawDelta(target: PhotonTrackedTarget): Rotation2d {
		// Inverted because vision conventions are CW positive, and
		// math conventions (which are  used in FRC) are CCW positive.
		val cameraToTargetYaw = -target.yaw.degrees
		return cameraToTargetYaw minus CAMERA_OFFSET
	}

	/**
	 *  Get the yaw delta between the robot and the best target note.
	 *
	 *  Returns `null` if there is no detected target.
	 */
	fun getRobotToBestTargetYawDelta(): Rotation2d? =
		bestTarget?.let { getRobotToTargetYawDelta(it) }

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Has targets", ::hasTargets, null)
		builder.addDoubleProperty(
			"Camera to target delta yaw",
			{
				bestTarget?.let {
					getRobotToTargetYawDelta(it).degrees
				} ?: -1.0
			},
			null
		)
	}
}