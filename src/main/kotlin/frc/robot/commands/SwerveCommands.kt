package frc.robot.commands

import com.hamosad1657.lib.commands.withName
import com.hamosad1657.lib.units.radPs
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.pow

fun SwerveSubsystem.teleopDriveCommand(
	vxSupplier: DoubleSupplier,
	vySupplier: DoubleSupplier,
	omegaSupplier: DoubleSupplier,
	isFieldRelative: BooleanSupplier,
) = withName("teleop drive") {
	run {
		val vx = -vxSupplier.asDouble.pow(3.0) * SwerveConstants.MAX_SPEED_MPS
		val vy = -vySupplier.asDouble.pow(3.0) * SwerveConstants.MAX_SPEED_MPS
		val omega = omegaSupplier.asDouble.pow(3.0) * SwerveConstants.MAX_ANGULAR_VELOCITY.radPs

		SmartDashboard.putNumber("vx", vx)
		SmartDashboard.putNumber("vy", vy)
		SmartDashboard.putNumber("omega", omega)

		// Drive using raw values.
		drive(
			Translation2d(vx, vy),
			omega.radPs,
			isFieldRelative.asBoolean,
			isOpenLoop = false,
		)
	}
}