package frc.robot.commands

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.math.mapRange
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.radPs
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Robot
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.shooter.DynamicShooting
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
	pidController: () -> PIDController = { Constants.CHASSIS_ANGLE_PID_CONTROLLER },
): Command = withName("teleop drive with auto angle") {
	run {
		pidController().setpoint = angleSupplier().degrees

		val vx = -vxSupplier().pow(3.0) * Constants.MAX_SPEED_MPS
		val vy = -vySupplier().pow(3.0) * Constants.MAX_SPEED_MPS
		val omega = Constants.CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)

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
			Constants.CHASSIS_ANGLE_PID_CONTROLLER.setpoint = setpoint.degrees
			setAngularVelocity(
				(Constants.CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)).radPs
			)
		}
}

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.getToAngleCommand(angle: () -> Rotation2d): Command = withName("get to angle command") {
	run {
		Constants.CHASSIS_ANGLE_PID_CONTROLLER.setpoint = angle().degrees
		setAngularVelocity(
			(Constants.CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)).radPs
		)
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Swerve.
 */
fun Swerve.getToAngleCommand(angle: Rotation2d): Command = withName("get to angle command") {
	run {
		Constants.CHASSIS_ANGLE_PID_CONTROLLER.setpoint = angle.degrees
		setAngularVelocity(
			(Constants.CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)).radPs
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
fun Swerve.driveToTrapCommand(): Command = withName("drive to trap") {
	val rotationSupplier: () -> Double = {
		if (ClimbingSubsystem.areBothTrapSwitchesPressed) 0.0
		else if (ClimbingSubsystem.isLeftTrapSwitchPressed) 0.3
		else if (ClimbingSubsystem.isRightTrapSwitchPressed) -0.3
		else 0.0
	}
	teleopDriveCommand({ 0.2 }, { 0.0 }, rotationSupplier, { false }) until
		{ ClimbingSubsystem.areBothTrapSwitchesPressed } andThen
		(teleopDriveCommand(
			{ -0.2 },
			{ 0.0 },
			{ 0.0 },
			{ false }) withTimeout (0.15)) finallyDo InstantCommand({ stop() })
}