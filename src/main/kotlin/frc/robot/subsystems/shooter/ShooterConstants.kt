package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.*
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.cos

object ShooterConstants {
	// TODO: Find velocity and angle tolerances for shooter
	val VELOCITY_TOLERANCE: AngularVelocity = 75.0.rpm
	val ANGLE_TOLERANCE = 3.0.degrees

	const val KEEP_AT_MAX_ANGLE_OUTPUT = 0.03
	const val KEEP_AT_MIN_ANGLE_OUTPUT = -0.03

	const val TIME_TO_ESCAPE_ANGLE_LOCK_SEC = 0.05
	const val ESCAPE_ANGLE_LOCK_OUTPUT = 0.4
	val ANGLE_FOR_INTAKE = 26.0.degrees

	val SHOOTER_MAX_VELOCITY = 5000.rpm

	val SHOOTER_PID_GAINS = PIDGains(
		0.0, 0.006, 0.0,
		kFF = { setpointRpm -> 0.0019 * setpointRpm },
		kIZone = 150.0,
	)

	val ANGLE_PID_GAINS = PIDGains(30.0, 0.0, 0.0)
	val ANGLE_MOTION_MAGIC_CONFIG = MotionMagicConfigs().apply {
		MotionMagicCruiseVelocity = 1.0
		MotionMagicAcceleration = 1.5
	}

	private val VERTICAL_CENTER_LINE_OFFSET = 5.degrees

	fun calculateAngleFF(currentAngle: Rotation2d): Volts {
		val cosine = cos(currentAngle.radians - VERTICAL_CENTER_LINE_OFFSET.radians)
		val ff = cosine * KEEP_AT_MIN_ANGLE_OUTPUT * 12.0

		val verticalCenter = 90.0 - VERTICAL_CENTER_LINE_OFFSET.degrees
		val hasPassedVerticalCenter = currentAngle.degrees > verticalCenter
		return if (hasPassedVerticalCenter) ff * 0.9 + 0.15 else ff
	}

	// Calculate the gear ratio.
	const val ANGLE_MOTOR_TO_CANCODER_GEAR_RATIO = (66.0 / 32.0) * 4 * 4

	const val ANGLE_CLOSED_LOOP_TELEOP_MULTIPLIER = 20.0

	/**
	 * Time between when loading started to when the note is shot.
	 * It might be a little different in different speeds, so put here
	 * it's maximum value.
	 */
	const val SHOOT_TIME_SEC = 2.0 // TODO: Measure SHOOT_TIME_SEC

	/**
	 * 1 degree should be the lowest possible angle.
	 * It should be 1 degree and not 0 so that it doesn't wrap to 360 by accident.
	 */
	val CANCODER_OFFSET = (-56.0).degrees

	/** This should eject the note quickly without getting it too far away. */
	const val EJECT_OUTPUT: PercentOutput = 0.0

	enum class AngleMotorDirection {
		TOWARDS_MIN,
		TOWARDS_MAX;
	}

	// ShooterState is a data class and not an enum, because we might want to make
	// a continuous shooting function if we have the time. In the meantime, we will
	// shoot from a few constant positions. Keep instances of ShooterState as constants.
	data class ShooterState(val angle: Rotation2d, val velocity: AngularVelocity) {

		companion object {
			// TODO: Test and find the shooter states
			val TO_AMP = ShooterState(Rotation2d(), 0.0.rpm)
			val TO_TRAP = ShooterState(Rotation2d(), 0.0.rpm)

			// TODO: Name these shooter states better
			val TO_SPEAKER_1 = ShooterState(Rotation2d(), 0.0.rpm)
			val TO_SPEAKER_2 = ShooterState(Rotation2d(), 0.0.rpm)
			val TO_SPEAKER_3 = ShooterState(Rotation2d(), 0.0.rpm)

			val COLLECT = ShooterState(ANGLE_FOR_INTAKE, 0.0.rpm)
		}
	}
}