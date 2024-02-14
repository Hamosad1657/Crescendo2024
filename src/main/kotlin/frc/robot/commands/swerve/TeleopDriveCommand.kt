package frc.robot.commands.swerve

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.myswerve.MySwerveSubsystem
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.math.pow
import frc.robot.subsystems.myswerve.MySwerveConstants as Constants

class TeleopDriveCommand(
	private val swerve: MySwerveSubsystem,
	private val vX: DoubleSupplier,
	private val vY: DoubleSupplier,
	private val omega: DoubleSupplier,
	private val isFieldRelative: BooleanSupplier,
	private val headingCorrection: Boolean
) : Command() {
	private val controller: SwerveController = swerve.swerveController
	private val timer = Timer()
	private var angle = 0.0
	private var lastTime = 0.0

	init {
		addRequirements(swerve)
	}

	override fun initialize() {
		if (headingCorrection) {
			timer.start()
			lastTime = timer.get()
		}
	}

	override fun execute() {
		val xVelocity = -vX.asDouble.pow(3.0)
		val yVelocity = -vY.asDouble.pow(3.0)
		val angleVelocity = omega.asDouble.pow(3.0)

		SmartDashboard.putNumber("vx", xVelocity)
		SmartDashboard.putNumber("vy", yVelocity)
		SmartDashboard.putNumber("omega", angleVelocity)

		if (headingCorrection) {
			// Estimate the desired angle in radians.
			val deltaTime = timer.get() - lastTime
			angle += angleVelocity * deltaTime * controller.config.maxAngularVelocity

			// Get the desired ChassisSpeeds given the desired angle and current angle.
			val correctedChassisSpeeds =
				controller.getTargetSpeeds(xVelocity, yVelocity, angle, swerve.heading.radians, Constants.MAX_SPEED)

			// Drive using given data points.
			swerve.drive(
				SwerveController.getTranslation2d(correctedChassisSpeeds),
				correctedChassisSpeeds.omegaRadiansPerSecond,
				isFieldRelative.asBoolean,
				false,
			)

			lastTime = timer.get()
		} else {
			// Drive using raw values.
			swerve.drive(
				Translation2d(xVelocity * Constants.MAX_SPEED, yVelocity * Constants.MAX_SPEED),
				angleVelocity * controller.config.maxAngularVelocity,
				isFieldRelative.asBoolean,
				false,
			)
		}
	}
}