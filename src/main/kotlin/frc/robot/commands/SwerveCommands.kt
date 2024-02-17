package frc.robot.commands

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.commands.withName
import com.hamosad1657.lib.units.radPs
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Robot
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import kotlin.math.pow
import kotlin.math.sign

fun SwerveSubsystem.teleopDriveCommand(
	vxSupplier: () -> Double,
	vySupplier: () -> Double,
	omegaSupplier: () -> Double,
	isFieldRelative: () -> Boolean,
	isClosedLoop: () -> Boolean = { false }
) = withName("teleop drive") {
	run {
		val vx = -vxSupplier().pow(3.0) * SwerveConstants.MAX_SPEED_MPS
		val vy = -vySupplier().pow(3.0) * SwerveConstants.MAX_SPEED_MPS
		val omega = -omegaSupplier().pow(4.0) * SwerveConstants.MAX_ANGULAR_VELOCITY.asRadPs * sign(omegaSupplier())

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
			isClosedLoop()
		)
	}
}