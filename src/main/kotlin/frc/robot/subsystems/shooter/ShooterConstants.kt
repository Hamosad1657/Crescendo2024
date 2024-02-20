package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.cos

object ShooterConstants {
	val ANGLE_CURRENT_LIMITS = CurrentLimitsConfigs().apply {
		SupplyCurrentLimitEnable = true
		SupplyCurrentLimit = 30.0
	}

	/**
	 * 1 degree should be the lowest possible angle.
	 * It should be 1 degree and not 0 so that it doesn't wrap to 360 by accident.
	 */
	val CANCODER_OFFSET = (-0.76625).rotations

	/** To set offset let the fall to its resting position while its on coast than subtract 90 degrees */
	val RESTING_ANGLE = 221.15.degrees
	val FLOOR_RELATIVE_OFFSET: Rotation2d = -(RESTING_ANGLE - 90.degrees)

	val VELOCITY_TOLERANCE: AngularVelocity = 50.0.rpm
	val SHOOTING_ANGLE_TOLERANCE = 1.0.degrees
	val AMP_ANGLE_TOLERANCE = 10.0.degrees

	const val KEEP_AT_MAX_ANGLE_OUTPUT = 0.03
	const val KEEP_AT_MIN_ANGLE_OUTPUT = 0.0

	const val TIME_TO_ESCAPE_ANGLE_LOCK_SEC = 0.1
	const val ESCAPE_ANGLE_LOCK_OUTPUT = -0.2

	val SHOOTER_PID_GAINS = PIDGains(
		0.0, 0.075, 0.0,
		kFF = { setpointRpm -> 0.0019 * setpointRpm },
		kIZone = 150.0,
	)

	val ANGLE_PID_GAINS = PIDGains(
		42.5, 0.0, 0.0,
	)
	val ANGLE_MOTION_MAGIC_CONFIG = MotionMagicConfigs().apply {
		MotionMagicCruiseVelocity = 2.0
		MotionMagicAcceleration = 4.0
	}

	private const val KEEP_PARALLEL_TO_FLOOR_OUTPUT = -0.0185

	fun calculateAngleFF(currentAngle: Rotation2d): Volts {
		val floorRelativeAngle = currentAngle + FLOOR_RELATIVE_OFFSET
		val ff = cos(floorRelativeAngle.radians) * KEEP_PARALLEL_TO_FLOOR_OUTPUT * 12.0
		return if (currentAngle.radians < RESTING_ANGLE.radians) ff * 1.125 - 0.1 else ff + 0.125
	}

	// Calculate the gear ratio.
	const val ANGLE_MOTOR_TO_CANCODER_GEAR_RATIO = (66.0 / 32.0) * 4 * 4

	const val ANGLE_CLOSED_LOOP_TELEOP_MULTIPLIER = 20.0

	const val SHOOTER_VOLTAGE_NEUTRAL_DEADBAND: Volts = 0.5
	const val SHOOTER_RAMP_RATE_SECONDS = 0.25

	/**
	 * Time between when loading started to when the note is shot.
	 * It might be a little different in different speeds, so put here it's maximum value.
	 */
	const val SHOOT_TIME_SEC = 1.0

	/**
	 * Time after which we load the note into the shooter no matter the velocity and angle error.
	 * It might be a little different in different speeds, so put here it's maximum value.
	 */
	const val SHOOT_TIMEOUT_SEC = 1.75

	/** This should eject the note quickly without getting it too far away. */
	const val EJECT_OUTPUT: PercentOutput = 0.3

	enum class AngleMotorDirection {
		TOWARDS_MIN,
		TOWARDS_MAX;
	}

	// ShooterState is a data class and not an enum, because we might want to make
	// a continuous shooting function if we have the time. In the meantime, we will
	// shoot from a few constant positions. Keep instances of ShooterState as constants.
	data class ShooterState(
		val angle: Rotation2d,
		val velocity: AngularVelocity,
	) {
		init {
			require(angle.degrees in 0.0..320.0) { "have a nice day :D" }
			require(velocity.asRpm in 0.0..6000.0) { "have a nice day :D" }
		}

		companion object {
			val COLLECT = ShooterState(168.degrees, 0.0.rpm)
			val TO_AMP = ShooterState(5.degrees, 0.0.rpm)
			val TO_TRAP = ShooterState(251.5.degrees, 2800.0.rpm)
			val EJECT = ShooterState(168.degrees, 1000.rpm)

			// TODO: Test and find the shooter states
			val AT_SPEAKER = ShooterState(200.degrees, 2600.rpm)
			val AT_STAGE = ShooterState(160.degrees, 4000.rpm)

			// TODO: Remove
			val COLLECT_TO_TRAP = ShooterState(270.degrees, 0.0.rpm)
		}
	}

	/**
	 * A map of pre-determined positions on the field to shooter states.
	 * To be used during autonomous period.
	 */
	val POSITION_STATE_MAP =
		mapOf(
			Pose2d() to ShooterState(0.0.degrees, 0.0.degPs),
			Pose2d() to ShooterState(0.0.degrees, 0.0.degPs)
		)
}