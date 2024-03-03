package frc.robot.subsystems.vision

import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Rotation2d
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

object NoteVision {
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
	val hasTarget: Boolean get() = latestResult?.hasTargets() ?: false

	/** Get the yaw delta between the robot and the target note. */
	fun getDeltaRobotToTargetYaw(target: PhotonTrackedTarget): Rotation2d {
		// Inverted because vision conventions are CW positive, and
		// math conventions (which are  used in FRC) are CCW positive.
		val cameraToTargetYaw = -target.yaw
		return (cameraToTargetYaw - CAMERA_OFFSET.degrees).degrees
	}


}