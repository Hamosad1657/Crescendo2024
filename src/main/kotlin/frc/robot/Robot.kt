package frc.robot

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.robotPrint
import com.hamosad1657.lib.units.degrees
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.*
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

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
	val isTesting = telemetryLevel == Telemetry.Testing

	private var autonomousCommand: Command? = null
	private var commandScheduler = CommandScheduler.getInstance().also {
		SmartDashboard.putData("commandScheduler", it)
	}

	/** This value is changed in [RobotContainer] using a [SendableChooser]. */
	var alliance = Alliance.Blue

	var submittedAuto: Command? = null

	override fun robotInit() {
		// Report the use of the Kotlin Language for "FRC Usage Report" statistics
		HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)

		// Access the RobotContainer object so that it is initialized. This will perform all our
		// button bindings, set default commands, and put our autonomous chooser on the dashboard.
		RobotContainer
	}

	override fun robotPeriodic() {
		commandScheduler.run()
	}

	override fun autonomousInit() {
		ShooterSubsystem.defaultCommand = ShooterSubsystem.autoDefaultCommand()
		autonomousCommand =
			ShooterSubsystem.escapeAngleLockCommand() andThen RobotContainer.getAutonomousCommand().asProxy()
		autonomousCommand?.schedule()
	}

	override fun autonomousExit() {
		if (alliance == Alliance.Red) {
			robotPrint("CHANGED GYRO ANGLE")
			SwerveSubsystem.setGyro((SwerveSubsystem.robotHeading.degrees - 180.0).degrees)
		}
	}

	override fun teleopInit() {
		ShooterSubsystem.defaultCommand = ShooterSubsystem.teleopDefaultCommand()
		autonomousCommand?.cancel()
	}

	override fun testInit() {
		// Cancels all running commands at the start of test mode.
		commandScheduler.cancelAll()
	}

	override fun disabledInit() {
		LoaderSubsystem.idleMode = IdleMode.kCoast
	}

	override fun disabledExit() {
		LoaderSubsystem.idleMode = IdleMode.kBrake
	}
}