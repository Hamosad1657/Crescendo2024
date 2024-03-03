package frc.robot.subsystems.shooter

import com.hamosad1657.lib.math.LinearInterpolationTable
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.math.mapRange
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Robot
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.vision.AprilTagVision

/**
 * - The position and distance units are meters.
 * - The angular units are degrees.
 * - The velocity units are RPM.
 */
object DynamicShooting {
	// --- Constants ---

	private const val MIN_DISTANCE_TO_SPEAKER = 2.5 // Meters
	private const val MAX_DISTANCE_TO_SPEAKER = 5.8 // Meters

	private const val MIN_ANGLE = 165.0 // Degrees
	private const val MAX_ANGLE = 180.0 // Degrees

	private const val MIN_VELOCITY = 3000.0 // RPM
	private const val MAX_VELOCITY = 4000.0 // RPM

	private val ANGLE_INTERPOLATION_TABLE =
		LinearInterpolationTable(
			0.0 to MAX_ANGLE,
			0.4146 to 169.0,
			1.0 to MIN_ANGLE,
		)


	// --- Calculations ---

	/** This function assumes the robot is directly facing the speaker. */
	fun calculateShooterState(robotPosition: Translation2d): ShooterState {
		val robotToSpeakerFlatDistance = robotPosition.getDistance(speakerPosition)
			.also { SmartDashboard.putNumber("Robot to speaker distance", it) }

		val distanceToSpeaker01 = distanceToSpeaker01(robotToSpeakerFlatDistance)
			.also { SmartDashboard.putNumber("Dynamic shooting factor", it) }

		return ShooterState(
			calculateAngleSetpoint(distanceToSpeaker01).degrees,
			calculateVelocitySetpoint(distanceToSpeaker01).rpm,
		)
	}

	/** Calculate the distance from the shooter as a 0.0->1.0 factor. */
	private fun distanceToSpeaker01(shooterToSpeakerFlatDistance: Double): Double {
		val distance = mapRange(
			value = shooterToSpeakerFlatDistance,
			startMin = MIN_DISTANCE_TO_SPEAKER,
			startMax = MAX_DISTANCE_TO_SPEAKER,
			endMin = 0.0,
			endMax = 1.0,
		)
		return clamp(distance, min = 0.0, max = 1.0)
	}

	/** Calculate the required shooter angle from the 0.0->1.0 distance factor. */
	private fun calculateAngleSetpoint(distanceToSpeaker01: Double): Double {
		val angle = ANGLE_INTERPOLATION_TABLE.getOutputFor(distanceToSpeaker01)
		return clamp(angle, MIN_ANGLE, MAX_ANGLE)
			.also { SmartDashboard.putNumber("Dynamic shooting angle", it) }
	}

	/** Calculate the required shooter setpoint from the 0.0->1.0 distance factor. */
	private fun calculateVelocitySetpoint(distanceToSpeaker01: Double): Double {
		val rpm = distanceToSpeaker01 * (MAX_VELOCITY - MIN_VELOCITY) + MIN_VELOCITY
		return clamp(rpm, MIN_VELOCITY, MAX_VELOCITY)
			.also { SmartDashboard.putNumber("Dynamic shooting velocity", it) }
	}


	// --- Speaker Info ---

	val isSpeakerTagDetected get() = AprilTagVision.getTag(speakerTagId) != null

	/** The Translation2d of the blue speaker if you are in the blue alliance, according to WPILib's field coordinate system. */
	private val SPEAKER_BLUE_POSITION_METERS = Translation2d(-0.04, 5.55) // Meters

	/** The Translation2d of the red speaker if you are in the red alliance, according to WPILib's field coordinate system. */
	private val SPEAKER_RED_POSITION_METERS = Translation2d(16.58, 5.55) // Meters

	/** ID of the april tag in the center of the speaker. */
	private const val SPEAKER_RED_TAG_ID = 4

	/** ID of the april tag in the center of the speaker. */
	private const val SPEAKER_BLUE_TAG_ID = 7

	val speakerPosition
		get() = when (Robot.alliance) {
			Alliance.Red -> SPEAKER_RED_POSITION_METERS
			Alliance.Blue -> SPEAKER_BLUE_POSITION_METERS
		}

	val speakerTagId
		get() = when (Robot.alliance) {
			Alliance.Red -> SPEAKER_RED_TAG_ID
			Alliance.Blue -> SPEAKER_BLUE_TAG_ID
		}
}

