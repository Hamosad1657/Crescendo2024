package frc.robot.subsystems.shooter

import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.robotAlliance
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState

/**
 * - The position and distance units are meters.
 * - The angular units are degrees.
 * - The velocity units are RPM.
 */
object DynamicShooting {
	private const val MIN_DISTANCE_TO_SPEAKER = 2.5 // Meters
	private const val MAX_DISTANCE_TO_SPEAKER = 6.5 // Meters

	private const val MIN_ANGLE = 144.0 // Degrees
	private const val MAX_ANGLE = 180.0 // Degrees

	private const val MIN_VELOCITY = 3000.0 // RPM
	private const val MAX_VELOCITY = 4000.0 // RPM

	private val speakerPosition = when (robotAlliance) {
		Alliance.Red -> SPEAKER_RED_POSITION_METERS
		Alliance.Blue -> SPEAKER_BLUE_POSITION_METERS
	}

	/** This function assumes the robot is directly facing the speaker. */
	fun calculateShooterState(robotPosition: Translation2d): ShooterState {
		val robotToSpeakerFlatDistance = robotPosition.getDistance(speakerPosition)
		val distanceToSpeaker01 = distanceToSpeaker01(robotToSpeakerFlatDistance)

		return ShooterState(
			calculateAngleSetpoint(distanceToSpeaker01).degrees,
			calculateVelocitySetpoint(distanceToSpeaker01).rpm,
		)
	}

	/** Calculate the distance from the shooter as a 0.0->1.0 factor. */
	private fun distanceToSpeaker01(shooterToSpeakerFlatDistance: Double): Double {
		val distance = (shooterToSpeakerFlatDistance - MIN_DISTANCE_TO_SPEAKER) / MAX_DISTANCE_TO_SPEAKER
		return clamp(distance, min = 0.0, max = 1.0)
	}

	/** Calculate the required shooter angle from the 0.0->1.0 distance factor. */
	private fun calculateAngleSetpoint(distanceToSpeaker01: Double): Double {
		val angle = (1.0 - distanceToSpeaker01) * (MAX_ANGLE - MIN_ANGLE) + MIN_ANGLE
		return clamp(angle, MIN_ANGLE, MAX_ANGLE)
	}

	/** Calculate the required shooter setpoint from the 0.0->1.0 distance factor. */
	private fun calculateVelocitySetpoint(distanceToSpeaker01: Double): Double {
		val rpm = distanceToSpeaker01 * (MAX_VELOCITY - MIN_VELOCITY) + MIN_VELOCITY
		return clamp(rpm, MIN_VELOCITY, MAX_VELOCITY)
	}
}

/**
 * The Translation2d of the blue speaker if you are in the blue alliance, according to WPILib's
 * field coordinate system.
 * Like last year, the field is not rotationally symmetrical, and we use the convention of "origin
 * follows alliance", so the speaker position will be different depending on our alliance.
 *
 * For more information:
 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#origin-follows-your-alliance
 */
private val SPEAKER_BLUE_POSITION_METERS = Translation2d(-0.04, 5.55) // Meters

/**
 * The Translation2d of the red speaker if you are in the red alliance, according to WPILib's
 * field coordinate system.
 * Like last year, the field is not rotationally symmetrical, and we use the convention of "origin
 * follows alliance", so the speaker position will be different depending on our alliance.
 *
 * For more information:
 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#origin-follows-your-alliance
 */
private val SPEAKER_RED_POSITION_METERS = Translation2d(16.58, 5.55) // Meters
