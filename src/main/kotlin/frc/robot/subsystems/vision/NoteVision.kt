package frc.robot.subsystems.vision

import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

object NoteVision : Sendable {
	/**
	 * The angle to the NOTE as reported by the vision,
	 * when the intake is perfectly centered at it.
	 */
	val CAMERA_OFFSET = 0.0.degrees

	private val camera: PhotonCamera? = try {
		PhotonCamera("NOTE-Cam")
	} catch (_: Exception) {
		null
	}

	val latestResult: PhotonPipelineResult? get() = camera?.latestResult
	val bestTarget: PhotonTrackedTarget? get() = latestResult?.bestTarget
	val hasTargets: Boolean get() = latestResult?.hasTargets() ?: false

	/** Get the yaw delta between the robot and the target note. */
	fun getDeltaRobotToTargetYaw(target: PhotonTrackedTarget): Rotation2d {
		// Inverted because vision conventions are CW positive, and
		// math conventions (which are  used in FRC) are CCW positive.
		val cameraToTargetYaw = -target.yaw
		return (cameraToTargetYaw - CAMERA_OFFSET.degrees).degrees
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.addBooleanProperty("Has targets", ::hasTargets, null)
		builder.addDoubleProperty(
			"Camera to target delta yaw",
			{ bestTarget?.let { getDeltaRobotToTargetYaw(it).degrees } ?: -1.0 },
			null
		)
	}
}