package frc.robot.subsystems.shooter

import com.hamosad1657.lib.math.*
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Robot
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.vision.Vision
import java.awt.geom.Point2D

/**
 * - The position and distance units are meters.
 * - The angular units are degrees.
 * - The velocity units are RPM.
 */
object DynamicShooting {
	private const val MIN_DISTANCE_TO_SPEAKER = 2.5 // Meters
	private const val MAX_DISTANCE_TO_SPEAKER = 5.8 // Meters

	private const val MIN_ANGLE = 162.0 // Degrees
	private const val MAX_ANGLE = 180.0 // Degrees

	private const val MIN_VELOCITY = 3000.0 // RPM
	private const val MAX_VELOCITY = 4000.0 // RPM

	private val ANGLE_INTERPOLATION_TABLE = LinearInterpolationTable(
		Point2D.Double(0.0, MAX_ANGLE),
		Point2D.Double(0.414600, 167.0),
		Point2D.Double(1.0, MIN_ANGLE)
	)

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

	val seesSpeakerTag
		get() = Vision.getTag(speakerTagId) != null


	/** This function assumes the robot is directly facing the speaker. */
	fun calculateShooterState(robotPosition: Translation2d): ShooterState {
		val robotToSpeakerFlatDistance = robotPosition.getDistance(speakerPosition)
		SmartDashboard.putNumber(
			"robot to speaker distance", robotToSpeakerFlatDistance
		)
		val distanceToSpeaker01 = distanceToSpeaker01(robotToSpeakerFlatDistance)

		SmartDashboard.putNumber("Dynamic shooting factor", distanceToSpeaker01)
		return ShooterState(
			calculateAngleSetpoint(distanceToSpeaker01).degrees,
			calculateVelocitySetpoint(distanceToSpeaker01).rpm,
		)
	}

	/** Calculate the distance from the shooter as a 0.0->1.0 factor. */
	private fun distanceToSpeaker01(shooterToSpeakerFlatDistance: Double): Double {
		val distance = mapRange(
			shooterToSpeakerFlatDistance,
			MIN_DISTANCE_TO_SPEAKER,
			MAX_DISTANCE_TO_SPEAKER,
			0.0,
			1.0
		)
		return clamp(distance, min = 0.0, max = 1.0)
	}

	/** Calculate the required shooter angle from the 0.0->1.0 distance factor. */
	private fun calculateAngleSetpoint(distanceToSpeaker01: Double): Double {
		//val factor = 1 - sqrt(distanceToSpeaker01)
//		val factor = 1 - distanceToSpeaker01
		val angle = ANGLE_INTERPOLATION_TABLE.getOutput(distanceToSpeaker01)
		SmartDashboard.putNumber(
			"Dynamic shooting angle",
			clamp(angle, MIN_ANGLE, MAX_ANGLE)
		)
		return clamp(angle, MIN_ANGLE, MAX_ANGLE)
	}

	/** Calculate the required shooter setpoint from the 0.0->1.0 distance factor. */
	private fun calculateVelocitySetpoint(distanceToSpeaker01: Double): Double {
		val rpm = distanceToSpeaker01 * (MAX_VELOCITY - MIN_VELOCITY) + MIN_VELOCITY
		return clamp(rpm, MIN_VELOCITY, MAX_VELOCITY)
	}

	/**
	 * The Translation2d of the blue speaker if you are in the blue alliance, according to WPILib's
	 * field coordinate system.
	 */
	private val SPEAKER_BLUE_POSITION_METERS = Translation2d(-0.04, 5.55) // Meters

	/**
	 * The Translation2d of the red speaker if you are in the red alliance, according to WPILib's
	 * field coordinate system.
	 */
	private val SPEAKER_RED_POSITION_METERS = Translation2d(16.58, 5.55) // Meters

	/**id of the april tag in the center of the speaker*/
	const val SPEAKER_RED_TAG_ID = 4

	/**id of the april tag in the center of the speaker*/
	const val SPEAKER_BLUE_TAG_ID = 7
}

