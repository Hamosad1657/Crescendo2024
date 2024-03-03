package frc.robot.commands

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.math.mapRange
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.plus
import com.hamosad1657.lib.units.radPs
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.subsystems.shooter.DynamicShooting
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveConstants.CHASSIS_ANGLE_PID_CONTROLLER
import frc.robot.subsystems.vision.NoteVision
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign
import frc.robot.subsystems.swerve.SwerveConstants as Constants
import frc.robot.subsystems.swerve.SwerveSubsystem as Swerve

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.crossLockWheelsCommand(): Command = withName("cross lock") {
	run { crossLockWheels() }
}

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.teleopDriveCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
	omegaSupplier: () -> Double,
	isFieldRelative: () -> Boolean,
	isClosedLoop: () -> Boolean = { true },
) = withName("teleop drive") {
	run {
		val vx = vxSupplier().let { it.pow(2.0) * Constants.MAX_SPEED_MPS * -it.sign }
		val vy = vySupplier().let { it.pow(2.0) * Constants.MAX_SPEED_MPS * -it.sign }
		val omega = omegaSupplier().let { it.pow(2.0) * Constants.MAX_ANGULAR_VELOCITY.asRadPs * -it.sign }

		if (Robot.telemetryLevel == Telemetry.Testing) {
			SmartDashboard.putNumber("vx", vx)
			SmartDashboard.putNumber("vy", vy)
			SmartDashboard.putNumber("omega", omega)
		}

		// Drive using raw values.
		drive(
			Translation2d(vx, vy),
			omega.radPs,
			isFieldRelative(),
			isClosedLoop(),
		)
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.teleopDriveWithAutoAngleCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
	angleSupplier: () -> Rotation2d,
	isFieldRelative: () -> Boolean,
	pidController: () -> PIDController = { CHASSIS_ANGLE_PID_CONTROLLER },
): Command = withName("teleop drive with auto angle") {
	run {
		pidController().setpoint = angleSupplier().degrees

		val vx = -vxSupplier().pow(3.0) * Constants.MAX_SPEED_MPS
		val vy = -vySupplier().pow(3.0) * Constants.MAX_SPEED_MPS
		val omega = CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)

		if (Robot.telemetryLevel == Telemetry.Testing) {
			SmartDashboard.putNumber("vx", vx)
			SmartDashboard.putNumber("vy", vy)
			SmartDashboard.putNumber("omega", omega)
		}

		// Drive using raw values.
		drive(
			Translation2d(vx, vy),
			omega.radPs,
			isFieldRelative(),
			true
		)
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.getToOneAngleCommand(angle: () -> Rotation2d): Command = withName("get to angle command") {
	var setpoint = 0.0.degrees
	runOnce {
		setpoint = angle()
	} andThen
		run {
			CHASSIS_ANGLE_PID_CONTROLLER.setpoint = setpoint.degrees
			setAngularVelocity(
				(CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)).radPs
			)
		}
}

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.getToAngleCommand(angle: () -> Rotation2d): Command = withName("get to angle command") {
	run {
		CHASSIS_ANGLE_PID_CONTROLLER.setpoint = angle().degrees
		setAngularVelocity(
			(CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)).radPs
		)
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.getToAngleCommand(angle: Rotation2d): Command = withName("get to angle command") {
	run {
		CHASSIS_ANGLE_PID_CONTROLLER.setpoint = angle.degrees
		setAngularVelocity(
			(CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)).radPs
		)
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.aimAtSpeakerWhileDrivingCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
): Command = teleopDriveWithAutoAngleCommand(
	vxSupplier,
	vySupplier,
	{
		val robotToGoal = robotPose.translation - DynamicShooting.speakerPosition
		mapRange(robotToGoal.angle.degrees, 0.0, 360.0, -180.0, 180.0).degrees

	},
	{ true },
)

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.aimAtGoalWhileDrivingCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
	goalSupplier: () -> Translation2d,
	isFieldRelative: () -> Boolean,
): Command = withName("aim at goal while driving") {
	teleopDriveWithAutoAngleCommand(
		vxSupplier,
		vySupplier,
		{
			val robotToGoal = robotPose.translation - goalSupplier()
			mapRange(robotToGoal.angle.degrees, 0.0, 360.0, -180.0, 180.0).degrees
		},
		isFieldRelative,
	)
}

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.aimAtNoteWhileDrivingCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
	omegaSupplier: () -> Double,
): Command = withName("aim at note while driving") {
	val joystickMovedWaitTimeSec = 0.5
	val joystickMoveTimer = Timer()
	var shouldFollowNote = true

	run {
		SmartDashboard.putNumber("timer", joystickMoveTimer.get())
		val joystickMoved = abs(omegaSupplier()) > RobotContainer.JOYSTICK_DEADBAND

		val omega =
			// The driver takes control of the rotation.
			if (joystickMoved or !NoteVision.hasTargets) {
				shouldFollowNote = false

				joystickMoveTimer.reset()
				joystickMoveTimer.start()

				-omegaSupplier().pow(3) * Constants.MAX_ANGULAR_VELOCITY.asRadPs
			}
			// If enough time has passed since the driver controlled the rotation,
			// go back to following the note using PID.
			else if (shouldFollowNote || joystickMoveTimer.hasElapsed(joystickMovedWaitTimeSec)) {
				shouldFollowNote = true
				joystickMoveTimer.stop()

				// The difference between the robot's angle and the detected Note.
				val rotationDelta = NoteVision.getRobotToBestTargetYawDelta() ?: 0.degrees

				// Calculate the required omega to rotate towards the Note using PID.
				val setpoint = (robotHeading plus rotationDelta).degrees
				SwerveConstants.CHASSIS_VISION_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees, setpoint)
			}
			// Stay at the same angle (do not rotate) for [joystickMovedWaitTimeSec] seconds.
			else {
				0.0
			}

		val vx = -vxSupplier().pow(3.0) * Constants.MAX_SPEED_MPS
		val vy = -vySupplier().pow(3.0) * Constants.MAX_SPEED_MPS

		// Drive using raw values.
		drive(
			Translation2d(vx, vy),
			omega.radPs,
			isFieldRelative = true,
			useClosedLoopDrive = true,
		)
	}
}