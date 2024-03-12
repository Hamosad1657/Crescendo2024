package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.robotPrintError
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Seconds
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.minus
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.cos

object ShooterConstants {
	// --- Configs ---

	val ANGLE_CURRENT_LIMITS =
		CurrentLimitsConfigs().apply {
			SupplyCurrentLimitEnable = true
			SupplyCurrentLimit = 30.0
		}

	/**
	 * 1 degree should be the lowest possible angle.
	 * It should be 1 degree and not 0 so that it doesn't wrap to 360 by accident.
	 */
	val CANCODER_OFFSET = (-274).degrees

	// Calculate the gear ratio.
	const val ANGLE_MOTOR_TO_CANCODER_GEAR_RATIO = (66.0 / 32.0) * 4 * 4

	const val SHOOTER_VOLTAGE_NEUTRAL_DEADBAND: Volts = 0.5
	const val SHOOTER_RAMP_RATE_SEC = 0.25


	// --- Constants ---

	val MAX_ANGLE = 293.degrees

	/** To set offset, let the shooter fall to its resting position while its on coast, then subtract 90 degrees. */
	private val RESTING_ANGLE = 223.5.degrees

	private val FLOOR_RELATIVE_OFFSET: Rotation2d = RESTING_ANGLE minus 90.degrees


	// --- Outputs ---

	const val KEEP_AT_MAX_ANGLE_OUTPUT: PercentOutput = 0.03
	const val KEEP_AT_MIN_ANGLE_OUTPUT: PercentOutput = 0.0

	const val ESCAPE_ANGLE_LOCK_OUTPUT: PercentOutput = -0.2
	const val ESCAPE_ANGLE_LOCK_DURATION: Seconds = 0.1

	private const val KEEP_PARALLEL_TO_FLOOR_OUTPUT = -0.0185

	/**
	 * Time between when loading started to when the note is shot.
	 * It might be a little different in different speeds, so put here it's maximum value.
	 */
	const val SHOOT_DURATION: Seconds = 1.0

	/**
	 * Time after which we load the note into the shooter no matter the velocity and angle error.
	 * It might be a little different in different speeds, so put here it's maximum value.
	 */
	const val SHOOT_TIMEOUT = 1.75


	// --- Tolerances ---

	val VELOCITY_TOLERANCE = 50.0.rpm
	val SHOOTING_ANGLE_TOLERANCE = 0.5.degrees
	val AMP_ANGLE_TOLERANCE = 20.0.degrees


	// --- PID Gains ---

	val SHOOTER_PID_GAINS = PIDGains(
		0.0003, 0.005, 0.0,
		kFF = { setpointRpm -> 0.0019 * setpointRpm },
		kIZone = 100.0,
	)

	val ANGLE_PID_GAINS = PIDGains(
		45.0, 0.0, 0.0,
//		0.0, 0.0, 0.0,
	)
	val ANGLE_MOTION_MAGIC_CONFIG = MotionMagicConfigs().apply {
		MotionMagicCruiseVelocity = 2.0
		MotionMagicAcceleration = 4.0
	}


	// --- Feedforward ---

	fun calculateAngleFF(currentAngle: Rotation2d): Volts {
		val floorRelativeAngle = currentAngle minus FLOOR_RELATIVE_OFFSET
		val ff = cos(floorRelativeAngle.radians) * KEEP_PARALLEL_TO_FLOOR_OUTPUT * 12.0
		return if (currentAngle.radians < RESTING_ANGLE.radians) ff * 1.125 - 0.1 else ff + 0.125
	}

	// --- Utility Classes ---

	enum class AngleMotorDirection {
		TOWARDS_MIN,
		TOWARDS_MAX;
	}

	// ShooterState is a data class and not an enum, because we might want to make
	// a continuous shooting function if we have the time. In the meantime, we will
	// shoot from a few constant positions. Keep instances of ShooterState as constants.
	class ShooterState(angle: Rotation2d, velocity: AngularVelocity) {
		val angle: Rotation2d
		val velocity: AngularVelocity

		init {
			this.angle =
				if (angle.degrees in 0.0..MAX_ANGLE.degrees) angle
				else {
					robotPrintError("Shooter angle out of bounds: ${angle.degrees}", true)
					0.degrees
				}

			this.velocity =
				if (velocity.asRpm in 0.0..5000.0) velocity
				else {
					robotPrintError("Shooter velocity out of bounds: ${angle.degrees}", true)
					0.rpm
				}
		}

		companion object {
			// --- Collection ---
			val COLLECT = ShooterState(175.degrees, 0.0.rpm)
			val AUTO_COLLECT = ShooterState(COLLECT.angle, 3000.rpm)
			val COLLECT_FROM_FEEDER = ShooterState(43.degrees, 0.0.rpm)

			// --- Teleop Speaker ---
			val AT_SPEAKER = ShooterState(200.degrees, 2600.rpm)
			val NEAR_SPEAKER = ShooterState(180.degrees, 3000.rpm)
			val AT_PODIUM = ShooterState(175.0.degrees, 3500.rpm)
			var AT_STAGE = ShooterState(162.85.degrees, 3900.rpm)

			// --- Teleop Misc. ---
			val TO_AMP = ShooterState(5.degrees, 0.0.rpm)
			val EJECT = ShooterState(168.degrees, 1000.rpm)
			val BEFORE_CLIMB = ShooterState(50.0.degrees, 0.0.rpm)

			// --- Auto ---
			val AUTO_LINE_ONE_THREE = ShooterState(176.degrees, 3500.rpm)
			val AUTO_LINE_TWO = ShooterState(178.degrees, 3500.rpm)
		}
	}
}