package frc.robot.commands

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.commands.withName
import com.hamosad1657.lib.math.mapRange
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.radPs
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Robot
import frc.robot.subsystems.shooter.DynamicShooting
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.vision.Vision
import kotlin.math.pow
import kotlin.math.sign
import frc.robot.subsystems.swerve.SwerveSubsystem as Swerve

fun Swerve.crossLockWheelsCommand(): Command = run { crossLockWheels() }

fun Swerve.teleopDriveCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
	omegaSupplier: () -> Double,
	isFieldRelative: () -> Boolean,
	isClosedLoop: () -> Boolean = { true }
) = withName("teleop drive") {
	run {
		val vx = vxSupplier().let { it.pow(2.0) * SwerveConstants.MAX_SPEED_MPS * -it.sign }
		val vy = vySupplier().let { it.pow(2.0) * SwerveConstants.MAX_SPEED_MPS * -it.sign }
		val omega = omegaSupplier().let { it.pow(2.0) * SwerveConstants.MAX_ANGULAR_VELOCITY.asRadPs * -it.sign }

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

fun Swerve.teleopDriveWithAutoAngleCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
	angleSupplier: () -> Rotation2d,
	isFieldRelative: () -> Boolean,
) = Swerve.run {
	SwerveConstants.CHASSIS_ANGLE_PID_CONTROLLER.setpoint = angleSupplier().degrees

	val vx = -vxSupplier().pow(3.0) * SwerveConstants.MAX_SPEED_MPS
	val vy = -vySupplier().pow(3.0) * SwerveConstants.MAX_SPEED_MPS
	val omega = SwerveConstants.CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)

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

fun Swerve.getToAngleCommand(angle: () -> Rotation2d): Command = withName("get to angle command") {
	run {
		SwerveConstants.CHASSIS_ANGLE_PID_CONTROLLER.setpoint = angle().degrees
		setAngularVelocity(
			(SwerveConstants.CHASSIS_ANGLE_PID_CONTROLLER.calculate(robotHeading.degrees)).radPs
		)
	}
}

fun Swerve.aimAtSpeakerWhileDrivingCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
	isFieldRelative: () -> Boolean,
): Command = teleopDriveWithAutoAngleCommand(
	vxSupplier,
	vySupplier,
	{
		if (DynamicShooting.seesSpeakerTag) {
			(robotHeading.degrees + Vision.getTag(DynamicShooting.speakerTagId)!!.yaw).degrees
		} else {
			val robotToGoal = robotPose.translation - DynamicShooting.speakerPosition
			mapRange(robotToGoal.angle.degrees, 0.0, 360.0, -180.0, 180.0).degrees
		}
	},
	isFieldRelative,
)

fun Swerve.aimAtGoalWhileDrivingCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
	goalSupplier: () -> Translation2d,
	isFieldRelative: () -> Boolean,
): Command = teleopDriveWithAutoAngleCommand(
	vxSupplier,
	vySupplier,
	{
		val robotToGoal = robotPose.translation - goalSupplier()
		mapRange(robotToGoal.angle.degrees, 0.0, 360.0, -180.0, 180.0).degrees
	},
	isFieldRelative,
)



