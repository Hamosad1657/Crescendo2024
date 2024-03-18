package frc.robot.subsystems.swerve

import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType.Velocity
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType.MotionMagic
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import com.hamosad1657.lib.robotPrintError
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.toNeutralModeValue
import com.hamosad1657.lib.vision.AprilTagCamera
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathPlannerPath
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.vision.AprilTagVision
import java.util.Optional
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveSubsystem : SwerveDrivetrain(
	Constants.DRIVETRAIN_CONSTANTS,
	Constants.Modules.FRONT_LEFT,
	Constants.Modules.FRONT_RIGHT,
	Constants.Modules.BACK_LEFT,
	Constants.Modules.BACK_RIGHT,
), Subsystem, Sendable {
	init {
		SendableRegistry.addLW(this, "SwerveSubsystem", "SwerveSubsystem")
		CommandScheduler.getInstance().registerSubsystem(this)
		configureAutoBuilder()
		configMotors()
	}

	override fun periodic() {
		if (AprilTagVision.FrontCam.isConnected) {
			addVisionMeasurement()
		}
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
	val robotHeading: Rotation2d get() = pigeon.rotation2d

	/** Gets the current pitch angle of the robot, as reported by the imu. */
	val robotPitch: Rotation2d get() = Rotation2d.fromDegrees(pigeon.pitch.valueAsDouble)

	/** Gets the current velocity (x, y and omega) of the robot. */
	val robotVelocity: ChassisSpeeds get() = kinematics.toChassisSpeeds(*modulesStates)

	/** Gets the current pose (position and rotation) of the robot, as reported by odometry. */
	val robotPose: Pose2d get() = poseEstimator.estimatedPosition

	val isFacingSpeaker: Boolean
		get() =
			currentState.Pose.rotation.degrees.let {
				if (Robot.alliance == Blue) it !in -90.0..90.0
				else it in -90.0..90.0
			}

	val isInVisionRange get() = AprilTagVision.FrontCam.isInRange


	// --- Drive & Module States Control

	private val controlRequestFieldRelative = SwerveRequest.FieldCentric().apply {
		Deadband = RobotContainer.JOYSTICK_DEADBAND * Constants.MAX_SPEED_MPS
		RotationalDeadband = RobotContainer.JOYSTICK_DEADBAND * Constants.MAX_ANGULAR_VELOCITY.asRadPs
	}
	private val controlRequestRobotRelative = SwerveRequest.RobotCentric().apply {
		Deadband = RobotContainer.JOYSTICK_DEADBAND * Constants.MAX_SPEED_MPS
		RotationalDeadband = RobotContainer.JOYSTICK_DEADBAND * Constants.MAX_ANGULAR_VELOCITY.asRadPs
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
	 * @param omega Robot angular rate (radians per second). CCW positive. Unaffected by field/robot relativity.
	 * @param isFieldRelative Drive mode. True for field-relative, false for robot-relative.
	 * @param useClosedLoopDrive Whether to use closed-loop velocity control. Set to true to enable closed-loop.
	 */
	fun drive(
		translation: Translation2d,
		omega: AngularVelocity,
		isFieldRelative: Boolean = true,
		useClosedLoopDrive: Boolean = false,
	) {
		if (isFieldRelative) {
			super.setControl(controlRequestFieldRelative.apply {
				VelocityX = translation.x * if (Robot.alliance == Alliance.Red) -1 else 1
				VelocityY = translation.y * if (Robot.alliance == Alliance.Red) -1 else 1
				RotationalRate = omega.asRadPs
				DriveRequestType = if (useClosedLoopDrive) Velocity else OpenLoopVoltage
				SteerRequestType = MotionMagic
			})
		} else {
			super.setControl(controlRequestRobotRelative.apply {
				VelocityX = translation.x
				VelocityY = translation.y
				RotationalRate = omega.asRadPs
				DriveRequestType = if (useClosedLoopDrive) Velocity else OpenLoopVoltage
				SteerRequestType = MotionMagic
			})
		}
	}

	private val controlRequestChassisSpeeds = SwerveRequest.ApplyChassisSpeeds()

	/** Stop the chassis from moving (set the module speeds to 0). */
	fun stop() {
		super.setControl(controlRequestChassisSpeeds.apply {
			Speeds = ChassisSpeeds(0.0, 0.0, 0.0)
		})
	}

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

	fun setAngularVelocity(angularVelocity: AngularVelocity) {
		super.setControl(controlRequestChassisSpeeds.apply {
			Speeds = ChassisSpeeds(0.0, 0.0, angularVelocity.asRadPs)
		})
	}

	private val controlRequestCrossLockWheels = SwerveRequest.SwerveDriveBrake()

	/** Lock the swerve drive (put the modules in a cross-like shape) to prevent it from moving. */
	fun crossLockWheels() {
		super.setControl(controlRequestCrossLockWheels)
	}


	// --- Motors Properties & Configuration

	private fun configMotors() {
		idleMode = IdleMode.kBrake
		for (module in super.Modules) {
			module.driveMotor.configurator.apply {
				apply(Constants.DRIVE_MOTOR_CONFIG.ClosedLoopRamps)
				apply(Constants.DRIVE_MOTOR_CONFIG.CurrentLimits)
			}
			module.steerMotor.configurator.apply {
				apply(Constants.STEER_MOTOR_CONFIG.CurrentLimits)
			}
		}
	}

	var idleMode: IdleMode = IdleMode.kBrake
		set(value) {
			super.configNeutralMode(value.toNeutralModeValue())
			field = value
		}


	// --- Odometry & Gyro ---

	/** Sets the gyroscope angle to 0. */
	fun zeroGyro() {
		pigeon.reset()
		poseEstimator.resetPosition(0.degrees, modulesPositions, robotPose.let { Pose2d(it.x, it.y, 0.degrees) })
	}

	/** Sets the expected gyroscope angle. */
	fun setGyro(angle: Rotation2d) {
		pigeon.setYaw(angle.degrees)
		poseEstimator.resetPosition(angle, modulesPositions, robotPose.let { Pose2d(it.x, it.y, angle) })
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
		setGyro(initialHolonomicPose.rotation)
		poseEstimator.resetPosition(robotHeading, modulesPositions, initialHolonomicPose)
		robotPrintError("Reset odometry to:  $initialHolonomicPose")
		robotPrintError("robot pose:  $robotPose")
		robotPrintError("robot heading: $robotHeading")
	}

	/** Update the odometry using the detected AprilTag (if any were detected). */
	private fun addVisionMeasurement() {
		fun addFrom(camera: AprilTagCamera) {
			// Don't update the position from the vision if:
			// - There is no pipeline result
			// - There is no detected AprilTag
			// - The robot is out of range.
			val latestResult = camera.latestResult ?: return
			latestResult.let {
				if (!it.hasTargets()) return
				if (!isInVisionRange) return
			}

			val estimatedPose = camera.estimatedGlobalPose
			if (estimatedPose != null) {
				field.getObject("vision_robot").pose = estimatedPose.estimatedPose.toPose2d()

				super.addVisionMeasurement(
					estimatedPose.estimatedPose.toPose2d().let { Pose2d(it.x, it.y, robotHeading) },
					estimatedPose.timestampSeconds,
					camera.poseEstimationStdDevs,
				)
			}
		}

		// The order matters for accuracy! Put last what you trust most.
		/*		if (Robot.alliance == Blue) {
					addFrom(AprilTagVision.RightCam)
					addFrom(AprilTagVision.LeftCam)
				} else {
					addFrom(AprilTagVision.LeftCam)
					addFrom(AprilTagVision.RightCam)
				} */
		addFrom(AprilTagVision.FrontCam)

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
				Robot.alliance == Alliance.Red
			},
			this
		)

		PPHolonomicDriveController.setRotationTargetOverride { Optional.empty() }
	}

	fun pathFindToPathCommand(pathname: String): Command {
		return AutoBuilder.pathfindThenFollowPath(
			PathPlannerPath.fromPathFile(pathname),
			SwerveConstants.PATH_CONSTRAINTS,
		)
	}

	fun pathfindToPoseCommand(pose: Pose2d): Command =
		AutoBuilder.pathfindToPose(pose, SwerveConstants.PATH_CONSTRAINTS)

	fun followAutoCommand(autoName: String): Command =
		AutoBuilder.buildAuto(autoName)

	fun followPathCommand(pathName: String): Command =
		AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))


	// --- Telemetry ---

	private val field = Field2d()

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("Subsystem")
		builder.addStringProperty("Command", { currentCommand?.name ?: "none" }, null)

		SmartDashboard.putData(field)
		super.registerTelemetry { state -> field.robotPose = state.Pose }

		builder.addDoubleProperty("Robot heading", { state.Pose.rotation.degrees }, null)
		builder.addStringProperty("Robot pose", { state.Pose.translation.toString() }, null)
		builder.addDoubleArrayProperty("Desired", { state.ModuleTargets.toDoubleArray() }, null)
		builder.addDoubleArrayProperty("Current", { state.ModuleStates.toDoubleArray() }, null)
	}

	private fun Array<SwerveModuleState>.toDoubleArray(): DoubleArray {
		val statesArray = Array(8) { 0.0 }
		for ((index, module) in this.withIndex()) {
			statesArray[index * 2] = module.angle.degrees
			statesArray[index * 2 + 1] = module.speedMetersPerSecond
		}
		return statesArray.toDoubleArray()
	}
}