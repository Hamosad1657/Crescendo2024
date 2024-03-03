package frc.robot.subsystems.vision

import com.hamosad1657.lib.units.degrees
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget

object NoteVision {

	private val camera: PhotonCamera? = try {
		PhotonCamera("NOTE-Cam")
	} catch (_: Exception) {
		null
	}

	val latestResult: PhotonPipelineResult? get() = camera?.latestResult
	val bestTarget: PhotonTrackedTarget? get() = latestResult?.bestTarget

	/** The yaw is negated because computer vision conventions are CW, and WPILib conventions are CCW. */
	fun getCameraToTargetYaw(target: PhotonTrackedTarget) = (-target.yaw).degrees
}