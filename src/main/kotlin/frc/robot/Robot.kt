package frc.robot

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.commands.andThen
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.*
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.vision.Vision

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : TimedRobot() {
	val telemetryLevel = Telemetry.Testing.also { SmartDashboard.putString("Telemetry", it.name) }

	private var autonomousCommand: Command? = null
	private var commandScheduler = CommandScheduler.getInstance()

	override fun robotInit() {
		// Report the use of the Kotlin Language for "FRC Usage Report" statistics
		HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)
		// Access the RobotContainer object so that it is initialized. This will perform all our
		// button bindings, set default commands, and put our autonomous chooser on the dashboard.
		Vision.estimatedGlobalPose?.let { SwerveSubsystem.setGyro(it.estimatedPose.rotation.toRotation2d()) }
		RobotContainer
	}

	override fun robotPeriodic() {
		commandScheduler.run()
	}

	override fun autonomousInit() {
		ShooterSubsystem.defaultCommand = ShooterSubsystem.autoDefaultCommand()
		autonomousCommand = ShooterSubsystem.escapeAngleLock() andThen RobotContainer.getAutonomousCommand()
		autonomousCommand?.schedule()
	}

	override fun teleopInit() {
		ShooterSubsystem.defaultCommand = ShooterSubsystem.teleopDefaultCommand()
		autonomousCommand?.cancel()
	}

	override fun testInit() {
		// Cancels all running commands at the start of test mode.
		commandScheduler.cancelAll()
	}
}