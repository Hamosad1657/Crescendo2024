package frc.robot.subsystems.shooter

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.*
import edu.wpi.first.math.geometry.Rotation2d

object ShooterConstants {
	// TODO: Find velocity and angle tolerances for shooter
	val VELOCITY_TOLERANCE: AngularVelocity = 0.0.rpm
	val ANGLE_TOLERANCE = 0.25.degrees

	val KEEP_AT_MAX_ANGLE_OUPTUT = 0.03
	val KEEP_AT_MIN_ANGLE_OUPTUT = -0.03

	val ANGLE_FOR_INTAKE = 35.41.degrees

	val ANGLE_PID_GAINS = PIDGains(22.0, 0.001, 0.0)

	// TODO: Check if this should be 1/20 or 20.
	const val ANGLE_MOTOR_TO_CANCODER_GEAR_RATIO = 20.0

	/**
	 * Time between when loading started to when the note is shot.
	 * It might be a little different in different speeds, so put here
	 * it's maximum value.
	 */
	const val SHOOT_TIME_SEC = 0.0 // TODO: Measure SHOOT_TIME_SEC

	/**
	 * 1 degree should be the lowest possible angle.
	 * It should be 1 degree and not 0 so that it doesn't wrap to 360 by accident.
	 */
	val CANCODER_OFFSET = (-0.15).rotations

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

			val COLLECT = ShooterState(Rotation2d(), 0.0.rpm)
		}
	}
}