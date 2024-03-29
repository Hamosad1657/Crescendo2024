package frc.robot.subsystems.swerve

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.inches
import com.hamosad1657.lib.units.meters
import com.hamosad1657.lib.units.rps
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Robot
import frc.robot.RobotMap.Swerve as SwerveMap

object SwerveConstants {
	// --- Constants ---

	/** Theoretical free speed (m/s) at 12v applied output. */
	const val MAX_SPEED_MPS = 9.0 // 9.46 according to CTRE ?

	/** Theoretical free rotation speed (rotations/s) at 12v applied output. */
	val MAX_ANGULAR_VELOCITY = 2.0.rps

	/** The distance from the center of the chassis to a center of a module. */
	private val DRIVEBASE_RADIUS = 0.417405.meters

	/**
	 * When the robot is adjacent and parallel to the PODIUM,
	 * rotate the chassis by this amount to score to the speaker.
	 */
	val AT_PODIUM_TO_SPEAKER_ROTATION
		get() =
			if (Robot.alliance == DriverStation.Alliance.Blue) (-25).degrees
			else 25.degrees

	val AT_STAGE_TO_SPEAKER_ROTATION
		get() =
			if (Robot.alliance == DriverStation.Alliance.Blue) 7.0.degrees
			else (-7.0).degrees


	// --- PID ---

	/** When using closed-loop control, the steer motor uses [ClosedLoopOutputType.Voltage]. */
	// TODO: Tune.
	private val STEER_PID_GAINS = Slot0Configs().apply {
		kP = 40.0; kI = 0.0; kD = 0.0
		kS = 0.0; kV = 1.5; kA = 0.0
	}

	/** When using closed-loop control, the drive motor uses [ClosedLoopOutputType.Voltage]. */
	private val DRIVE_PID_GAINS = Slot0Configs().apply {
		kP = 0.09; kI = 0.0; kD = 0.0
		kS = 0.0; kV = 0.12; kA = 0.0
	}

	// Good for TPU wheels!

//	/** Feedback from gyro, setpoint from anywhere. */
//	val CHASSIS_ANGLE_PID_CONTROLLER =
//		PIDController(0.12, 0.0, 0.0).apply {
//			enableContinuousInput(-180.0, 180.0)
//		}
//
//	/** Feedback from gyro, setpoint from vision! */
//	val CHASSIS_VISION_ANGLE_PID_CONTROLLER =
//		PIDController(0.12, 0.0, 0.0).apply {
//			enableContinuousInput(-180.0, 180.0)
//		}


	// Good for regular wheels!

	/** Feedback from gyro, setpoint from anywhere. */
	val CHASSIS_ANGLE_PID_CONTROLLER =
		PIDController(0.25, 0.0, 0.005).apply {
			enableContinuousInput(-180.0, 180.0)
		}

	val CHASSIS_AIM_AT_NOTE_PID_CONTROLLER =
		PIDController(0.17, 0.0, 0.0).apply {
			enableContinuousInput(-180.0, 180.0)
		}


	// --- PathPlanner ---

	private val PATH_TRANSLATION_CONSTANTS = PIDConstants(40.0, 0.0, 0.0)
	private val PATH_ROTATION_CONSTANTS = PIDConstants(25.0, 0.0, 0.00)

	// TODO: Tune.
	val PATH_CONSTRAINTS = PathConstraints(
		MAX_SPEED_MPS, // Max velocity (meters per second)
		MAX_SPEED_MPS / 2, // Max acceleration - 2 seconds to max velocity
		MAX_ANGULAR_VELOCITY.asRadPs, // Max angular velocity (radians per second)
		MAX_ANGULAR_VELOCITY.asRadPs * 2, // Max angular acceleration - 1 second to max velocity
	)

	val PATH_PLANNER_CONFIG = HolonomicPathFollowerConfig(
		PATH_TRANSLATION_CONSTANTS,
		PATH_ROTATION_CONSTANTS,
		MAX_SPEED_MPS,
		DRIVEBASE_RADIUS.asMeters,
		ReplanningConfig(),
	)


	// --- Configs ---

	val DRIVETRAIN_CONSTANTS = SwerveDrivetrainConstants().apply {
		Pigeon2Id = SwerveMap.PIGEON_ID
		CANbusName = SwerveMap.CANBUS_NAME
	}

	val STEER_MOTOR_CONFIG =
		TalonFXConfiguration().apply {
			CurrentLimits.apply {
				SupplyCurrentLimit = 20.0
				SupplyCurrentLimitEnable = true
			}
		}

	val DRIVE_MOTOR_CONFIG =
		TalonFXConfiguration().apply {
			CurrentLimits.apply {
				SupplyCurrentLimit = 45.0
				SupplyCurrentLimitEnable = true
				StatorCurrentLimitEnable = false
			}
			ClosedLoopRamps.apply {
				VoltageClosedLoopRampPeriod = 0.25
			}
		}

	/** How many rotations the drive motor does when the module rotates 1 rotation. */
	private const val COUPLING_GEAR_RATIO = 3.5714285714285716

	private val CONSTANT_CREATOR = SwerveModuleConstantsFactory().apply {
		WheelRadius = 2.0 // In inches.
		SlipCurrent = 300.0 // TODO: Tune.
		SteerMotorGearRatio = 12.8
		DriveMotorGearRatio = 6.746031746031747
		SteerMotorClosedLoopOutput = ClosedLoopOutputType.Voltage
		DriveMotorClosedLoopOutput = ClosedLoopOutputType.Voltage
		FeedbackSource = SteerFeedbackType.RemoteCANcoder
		SteerMotorInverted = false
		SpeedAt12VoltsMps = MAX_SPEED_MPS
		SteerMotorGains = STEER_PID_GAINS
		DriveMotorGains = DRIVE_PID_GAINS
		CouplingGearRatio = COUPLING_GEAR_RATIO
	}

	object Modules {
		private val DISTANCE_TO_MODULE_X = 11.62.inches
		private val DISTANCE_TO_MODULE_Y = 11.62.inches

		private const val SHOULD_INVERT_LEFT_SIDE = false
		private const val SHOULD_INVERT_RIGHT_SIDE = true

		private const val FRONT_LEFT_ENCODER_OFFSET = -0.363281
		private const val FRONT_RIGHT_ENCODER_OFFSET = -0.003906
		private const val BACK_LEFT_ENCODER_OFFSET = -0.489746
		private const val BACK_RIGHT_ENCODER_OFFSET = 0.232178

		/** FL, FR, BL, BR. */
		val asArray get() = arrayOf(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT)

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
