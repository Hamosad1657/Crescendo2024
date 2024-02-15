package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType.Velocity
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.hamosad1657.lib.*
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.toNeutralModeValue
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveSubsystem : SwerveDrivetrain(
	Constants.DRIVETRAIN_CONSTANTS,
	Constants.Modules.FRONT_LEFT,
	Constants.Modules.FRONT_RIGHT,
	Constants.Modules.BACK_LEFT,
	Constants.Modules.BACK_RIGHT,
), Subsystem {
	init {
		configureAutoBuilder()
	}

	override fun periodic() {
		poseEstimator.update(robotHeading, modulesPositions)
		addVisionMeasurement()
	}


	// --- Super-Class Members Aliases ---

	private inline val kinematics: SwerveDriveKinematics get() = super.m_kinematics
	private inline val poseEstimator: SwerveDrivePoseEstimator get() = super.m_odometry
	private inline val pigeon: Pigeon2 get() = super.getPigeon2()
	private inline val currentState: SwerveDriveState get() = super.getState()
	private inline val modulesStates: Array<SwerveModuleState> get() = currentState.ModuleStates
	private inline val modulesPositions: Array<SwerveModulePosition> get() = super.m_modulePositions

	// --- Robot State Getters ---

	/** Gets the current yaw angle of the robot, as reported by the IMU (CCW positive, not wrapped). */
	val robotHeading: Rotation2d get() = Rotation2d.fromDegrees(pigeon.yaw.valueAsDouble)

	/** Gets the current pitch angle of the robot, as reported by the imu. */
	val robotPitch: Rotation2d get() = Rotation2d.fromDegrees(pigeon.pitch.valueAsDouble)

	/** Gets the current velocity (x, y and omega) of the robot. */
	val robotVelocity: ChassisSpeeds get() = kinematics.toChassisSpeeds(*modulesStates)

	/** Gets the current pose (position and rotation) of the robot, as reported by odometry. */
	val robotPose: Pose2d get() = state.Pose


	// --- Drive & Module States Control

	private val controlRequestFieldRelative = SwerveRequest.FieldCentric().apply {
		Deadband = RobotContainer.JOYSTICK_DEADBAND
		RotationalDeadband = RobotContainer.JOYSTICK_DEADBAND
	}
	private val controlRequestRobotRelative = SwerveRequest.RobotCentric().apply {
		Deadband = RobotContainer.JOYSTICK_DEADBAND
		RotationalDeadband = RobotContainer.JOYSTICK_DEADBAND
	}

	/**
	 * The primary method for controlling the drivebase.
	 *
	 * - Takes a [Translation2d] and a rotation rate, and calculates and commands module states accordingly.
	 * - Can use either open-loop or closed-loop velocity control for the wheel velocities.
	 * - Also has field-relative and robot-relative modes, which affect how the translation vector is used.
	 *
	 * @param translation The commanded linear velocity of the robot, in meters per second.
	 * In robot-relative mode, positive x is towards the front and positive y is towards the left.
	 * In field-relative mode, positive x is away from the alliance wall (field North) and
	 * positive y is towards the left wall when looking through the driver station glass (field West).
	 * @param omega Robot angular rate. CCW positive. Unaffected by field/robot relativity.
	 * @param isFieldRelative Drive mode. True for field-relative, false for robot-relative.
	 * @param isOpenLoop Whether to use closed-loop velocity control. Set to true to disable closed-loop.
	 */
	fun drive(
		translation: Translation2d,
		omega: AngularVelocity,
		isFieldRelative: Boolean = true,
		isOpenLoop: Boolean = false
	) {
		if (isFieldRelative) {
			super.setControl(controlRequestFieldRelative.apply {
				VelocityX = translation.x
				VelocityY = translation.y
				RotationalRate = omega.asRadPs
				DriveRequestType = if (isOpenLoop) OpenLoopVoltage else Velocity
			})
		} else {
			super.setControl(controlRequestRobotRelative.apply {
				VelocityX = translation.x
				VelocityY = translation.y
				RotationalRate = omega.asRadPs
				DriveRequestType = if (isOpenLoop) OpenLoopVoltage else Velocity
			})
		}
	}

	private val controlRequestChassisSpeeds = SwerveRequest.ApplyChassisSpeeds()

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
		super.setControl(controlRequestChassisSpeeds.apply {
			Speeds = chassisSpeeds
		})
	}

	private val controlRequestCrossLockWheels = SwerveRequest.SwerveDriveBrake()

	/** Lock the swerve drive to prevent it from moving. */
	fun crossLockWheels() {
		super.setControl(controlRequestCrossLockWheels)
	}


	// --- Motors Properties & Configuration

	var idleMode: IdleMode = IdleMode.kBrake
		set(value) {
			super.configNeutralMode(value.toNeutralModeValue())
			field = value
		}


	// --- Odometry & Gyro ---

	/** Sets the gyroscope angle to 0. */
	fun zeroGyro() {
		pigeon.reset()
	}

	/** Sets the expected gyroscope angle. */
	fun setGyro(angle: Rotation2d) {
		pigeon.setYaw(angle.degrees)
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module
	 * positions do not need to be reset when calling this method.
	 *
	 * However, if either gyro angle or module position is reset,
	 * this must be called in order for odometry to keep working.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to.
	 */
	fun resetOdometry(initialHolonomicPose: Pose2d) {
		val modulePositions = Array(4) { SwerveModulePosition() }
		poseEstimator.resetPosition(robotHeading, modulePositions, initialHolonomicPose)
	}

	/** Update the odometry using the detected AprilTag (if any were detected). */
	private fun addVisionMeasurement() {
		Vision.estimatedPose2d?.let { estimatedPose ->
			val poseDelta = estimatedPose.relativeTo(robotPose)
			if (
				poseDelta.x < Vision.MAX_VISION_TO_ODOMETRY_DELTA.asMeters &&
				poseDelta.y < Vision.MAX_VISION_TO_ODOMETRY_DELTA.asMeters
			) {
				super.addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp())
			}
		}
	}


	// --- Auto & Paths ---

	private fun configureAutoBuilder() {
		AutoBuilder.configureHolonomic(
			::robotPose,
			::resetOdometry,
			::robotVelocity,
			::setChassisSpeeds,
			Constants.PATH_PLANNER_CONFIG,
			{
				// Boolean supplier that controls when the path will be mirrored for the red alliance
				// This will flip the path being followed to the red side of the field.
				// THE ORIGIN WILL REMAIN ON THE BLUE SIDE
				robotAlliance == DriverStation.Alliance.Red
			},
			this
		)
	}

	fun pathFindToPathCommand(pathname: String): Command {
		return AutoBuilder.pathfindThenFollowPath(
			PathPlannerPath.fromPathFile(pathname),
			SwerveConstants.PATH_CONSTRAINTS,
		)
	}

	fun pathFindToPoseCommand(pose: Pose2d): Command {
		return AutoBuilder.pathfindToPose(pose, SwerveConstants.PATH_CONSTRAINTS)
	}

	fun followAutoCommand(autoName: String): Command {
		return AutoBuilder.buildAuto(autoName)
	}

	fun followPathCommand(pathName: String): Command {
		return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))
	}


	// --- Telemetry ---

	private val telemetry = SwerveDriveTelemetry()

	init {
		when (Robot.telemetryLevel) {
			Telemetry.Testing -> {
				super.registerTelemetry { state ->
					telemetry.telemeterize(state)

					// Also a possibility
					// customTelemetry(state)
				}
			}

			else -> {}
		}
	}

	private fun customTelemetry(state: SwerveDriveState) {
		val field = Field2d()
		SmartDashboard.putData(field)

		// Current module states
		state.ModuleStates.forEachIndexed { index, module ->
			SmartDashboard.putData(module.toSendable(index, prefix = "Current/"))
		}

		// Desired module states
		state.ModuleTargets.forEachIndexed { index, module ->
			SmartDashboard.putData(module.toSendable(index, prefix = "Desired/"))
		}

		// Robot pose (x, y, omega)
		SmartDashboard.putData("RobotPose", state.Pose.toSendable())
		field.robotPose = state.Pose
	}
}