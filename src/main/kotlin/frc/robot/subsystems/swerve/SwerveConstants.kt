package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.mechanisms.swerve.*
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType
import com.hamosad1657.lib.units.*
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.*
import frc.robot.RobotMap.Swerve as SwerveMap

object SwerveConstants {
	val DRIVETRAIN_CONSTANTS = SwerveDrivetrainConstants().apply {
		Pigeon2Id = SwerveMap.PIGEON_ID
		CANbusName = SwerveMap.CANBUS_NAME
	}

	/** Theoretical free speed (m/s) at 12v applied output. */
	const val MAX_SPEED_MPS = 5.0 // 9.46 according to CTRE ?

	/** Theoretical free rotation speed (rotations/s) at 12v applied output. */
	val MAX_ANGULAR_VELOCITY = 1.5.rps

	/** When using closed-loop control, the drive motor uses [ClosedLoopOutputType.Voltage]. */
	// TODO: Tune.
	private val DRIVE_PID_GAINS = Slot0Configs().apply {
		kP = 3.0; kI = 0.0; kD = 0.0
		kS = 0.0; kV = 0.0; kA = 0.0
	}

	/** When using closed-loop control, the steer motor uses [ClosedLoopOutputType.Voltage]. */
	// TODO: Tune.
	private val STEER_PID_GAINS = Slot0Configs().apply {
		kP = 100.0; kI = 0.0; kD = 0.2
		kS = 0.0; kV = 1.5; kA = 0.0
	}

	/** How many rotations the drive motor does when the module rotates 1 rotation. */
	private const val COUPLING_GEAR_RATIO = 3.5714285714285716

	private val CONSTANT_CREATOR = SwerveModuleConstantsFactory().apply {
		DriveMotorGearRatio = 6.746031746031747
		SteerMotorGearRatio = 12.8
		WheelRadius = 4.0 // In inches.
		SlipCurrent = 300.0 // TODO: Tune.
		SteerMotorClosedLoopOutput = ClosedLoopOutputType.Voltage
		DriveMotorClosedLoopOutput = ClosedLoopOutputType.Voltage
		FeedbackSource = SteerFeedbackType.RemoteCANcoder
		SteerMotorInverted = false
		SpeedAt12VoltsMps = MAX_SPEED_MPS
		SteerMotorGains = STEER_PID_GAINS
		DriveMotorGains = DRIVE_PID_GAINS
		CouplingGearRatio = COUPLING_GEAR_RATIO
	}

	// TODO: Tune.
	val PATH_CONSTRAINTS = PathConstraints(
		MAX_SPEED_MPS, // Max velocity (meters per second)
		MAX_SPEED_MPS / 2, // Max acceleration - 2 seconds to max velocity
		MAX_ANGULAR_VELOCITY.asRadPs, // Max angular velocity (radians per second)
		MAX_ANGULAR_VELOCITY.asRadPs * 2, // Max angular acceleration - 1 second to max velocity
	)

	private val PATH_TRANSLATION_CONSTANTS = PIDConstants(13.0, 1.0, 1.0)
	private val PATH_ROTATION_CONSTANTS = PIDConstants(6.0, 0.0, 0.0)

	private val DRIVEBASE_RADIUS = 0.417405.meters

	val PATH_PLANNER_CONFIG = HolonomicPathFollowerConfig(
		PATH_TRANSLATION_CONSTANTS,
		PATH_ROTATION_CONSTANTS,
		MAX_SPEED_MPS,
		DRIVEBASE_RADIUS.asMeters,
		ReplanningConfig(),
	)

	object Modules {
		private val DISTANCE_TO_MODULE_X = 11.62.inches
		private val DISTANCE_TO_MODULE_Y = 11.62.inches

		private const val SHOULD_INVERT_LEFT_SIDE = false
		private const val SHOULD_INVERT_RIGHT_SIDE = true

		private const val FRONT_LEFT_ENCODER_OFFSET = -0.710693359375
		private const val FRONT_RIGHT_ENCODER_OFFSET = -0.005859375
		private const val BACK_LEFT_ENCODER_OFFSET = -0.478759765625
		private const val BACK_RIGHT_ENCODER_OFFSET = -0.7646484375

		val FRONT_LEFT: SwerveModuleConstants = CONSTANT_CREATOR.createModuleConstants(
			SwerveMap.FrontLeft.STEER_MOTOR_ID,
			SwerveMap.FrontLeft.DRIVE_MOTOR_ID,
			SwerveMap.FrontLeft.CANCODER_ID,
			FRONT_LEFT_ENCODER_OFFSET,
			DISTANCE_TO_MODULE_X.asMeters,
			DISTANCE_TO_MODULE_Y.asMeters,
			SHOULD_INVERT_LEFT_SIDE,
		)

		val FRONT_RIGHT: SwerveModuleConstants = CONSTANT_CREATOR.createModuleConstants(
			SwerveMap.FrontRight.STEER_MOTOR_ID,
			SwerveMap.FrontRight.DRIVE_MOTOR_ID,
			SwerveMap.FrontRight.CANCODER_ID,
			FRONT_RIGHT_ENCODER_OFFSET,
			DISTANCE_TO_MODULE_X.asMeters,
			-DISTANCE_TO_MODULE_Y.asMeters,
			SHOULD_INVERT_RIGHT_SIDE,
		)

		val BACK_LEFT: SwerveModuleConstants = CONSTANT_CREATOR.createModuleConstants(
			SwerveMap.BackLeft.STEER_MOTOR_ID,
			SwerveMap.BackLeft.DRIVE_MOTOR_ID,
			SwerveMap.BackLeft.CANCODER_ID,
			BACK_LEFT_ENCODER_OFFSET,
			-DISTANCE_TO_MODULE_X.asMeters,
			DISTANCE_TO_MODULE_Y.asMeters,
			SHOULD_INVERT_LEFT_SIDE,
		)

		val BACK_RIGHT: SwerveModuleConstants = CONSTANT_CREATOR.createModuleConstants(
			SwerveMap.BackRight.STEER_MOTOR_ID,
			SwerveMap.BackRight.DRIVE_MOTOR_ID,
			SwerveMap.BackRight.CANCODER_ID,
			BACK_RIGHT_ENCODER_OFFSET,
			-DISTANCE_TO_MODULE_X.asMeters,
			-DISTANCE_TO_MODULE_Y.asMeters,
			SHOULD_INVERT_RIGHT_SIDE,
		)
	}
}
