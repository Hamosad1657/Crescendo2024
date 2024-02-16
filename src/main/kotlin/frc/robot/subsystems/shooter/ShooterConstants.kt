package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.*
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.cos

object ShooterConstants {
	// TODO: Find velocity and angle tolerances for shooter
	val VELOCITY_TOLERANCE: AngularVelocity = 10.0.rpm
	val ANGLE_TOLERANCE = 1.5.degrees

	const val KEEP_AT_MAX_ANGLE_OUTPUT = 0.03
	const val KEEP_AT_MIN_ANGLE_OUTPUT = -0.03

	// TODO: Test this
	const val TIME_TO_ESCAPE_ANGLE_LOCK_SEC = 0.02
	const val ESCAPE_ANGLE_LOCK_OUTPUT = -0.3
	
	val ANGLE_FOR_INTAKE = 122.degrees // 42.0.degrees

	val ANGLE_FOR_AMP = (-120.0).degrees

	val SHOOTER_PID_GAINS = PIDGains(
		0.0, 0.006, 0.0,
		kFF = { setpointRpm -> 0.0019 * setpointRpm },
		kIZone = 150.0,
	)

	val ANGLE_PID_GAINS = PIDGains(45.0, 0.0, 0.0)
	val ANGLE_MOTION_MAGIC_CONFIG = MotionMagicConfigs().apply {
		MotionMagicCruiseVelocity = 0.8
		MotionMagicAcceleration = 2.0
	}

	fun calculateAngleFF(currentAngle: Rotation2d): Volts {
		val ff = cos(currentAngle.radians) * KEEP_AT_MIN_ANGLE_OUTPUT * 12.0 * 0.7
		return if (currentAngle.degrees < 0.0) ff * 0.7 else ff
	}

	// Calculate the gear ratio.
	const val ANGLE_MOTOR_TO_CANCODER_GEAR_RATIO = (66.0 / 32.0) * 4 * 4

	const val ANGLE_CLOSED_LOOP_TELEOP_MULTIPLIER = 20.0

	/**
	 * Time between when loading started to when the note is shot.
	 * It might be a little different in different speeds, so put here
	 * it's maximum value.
	 */
	const val SHOOT_TIME_SEC = 1.25

	/**
	 * 1 degree should be the lowest possible angle.
	 * It should be 1 degree and not 0 so that it doesn't wrap to 360 by accident.
	 */
	val CANCODER_OFFSET = (-0.1254).rotations

	/** This should eject the note quickly without getting it too far away. */
	const val EJECT_OUTPUT: PercentOutput = 0.3

	enum class AngleMotorDirection {
		TOWARDS_MIN,
		TOWARDS_MAX;
	}

	// ShooterState is a data class and not an enum, because we might want to make
	// a continuous shooting function if we have the time. In the meantime, we will
	// shoot from a few constant positions. Keep instances of ShooterState as constants.
	data class ShooterState(val angle: Rotation2d, val velocity: AngularVelocity) {
		init {
			require(angle.degrees in 0.0..320.0) { "have a nice day :D" }
			require(velocity.asRpm in 0.0..6000.0) { "have a nice day :D" }
		}

		companion object {
			// TODO: Test and find the shooter states
			val TO_TRAP = ShooterState(122.0.degrees, 2300.0.rpm)

			// TODO: Name these shooter states better
			val TO_SPEAKER_1 = ShooterState(Rotation2d(), 0.0.rpm)
			val TO_SPEAKER_2 = ShooterState(Rotation2d(), 0.0.rpm)
			val TO_SPEAKER_3 = ShooterState(Rotation2d(), 0.0.rpm)

			val COLLECT = ShooterState(ANGLE_FOR_INTAKE, 0.0.rpm)
		}
	}
}