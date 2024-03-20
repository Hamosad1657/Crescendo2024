package frc.robot

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.robotPrint
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
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.DEFAULT
import frc.robot.vision.AprilTagVision
import frc.robot.vision.NoteVision
import frc.robot.subsystems.intake.IntakeSubsystem as Intake
import frc.robot.subsystems.leds.LEDsSubsystem as LEDs
import frc.robot.subsystems.loader.LoaderSubsystem as Loader
import frc.robot.subsystems.shooter.ShooterSubsystem as Shooter

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : TimedRobot() {
	val telemetryLevel =
		Telemetry.Competition
			.also { SmartDashboard.putString("Telemetry", it.name) }

	val isTesting = telemetryLevel == Telemetry.Testing

	/** This value is changed in [RobotContainer] using a [SendableChooser]. */
	var alliance = Alliance.Blue

	private var autonomousCommand: Command? = null
	private var commandScheduler =
		CommandScheduler.getInstance()
			.also { SmartDashboard.putData("commandScheduler", it) }

	override fun robotInit() {
		// Report the use of the Kotlin Language for "FRC Usage Report" statistics
		HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version)

		// Access the RobotContainer object so that it is initialized. This will perform all our
		// button bindings, set default commands, and put our autonomous chooser on the dashboard.
		RobotContainer
	}

	override fun robotPeriodic() {
		commandScheduler.run()

		// Call periodically so the alert updates in Shuffleboard
		AprilTagVision.FrontCam.isConnected
		NoteVision.isConnected
	}

	override fun autonomousInit() {
		autonomousCommand =
			Shooter.escapeAngleLockCommand() andThen
				RobotContainer.getAutonomousCommand()
					.also { robotPrint("Auto command: ${it.name}") }
					.asProxy()

		autonomousCommand?.schedule()

		Shooter.defaultCommand = Shooter.autoDefaultCommand()
	}

	override fun teleopInit() {
		autonomousCommand?.cancel()

		Shooter.defaultCommand =
			LEDs::setToDefaultMode.asInstantCommand andThen
				Shooter.teleopDefaultCommand()
	}

	override fun disabledInit() {
		Loader.idleMode = IdleMode.kCoast
		Intake.idleMode = IdleMode.kCoast
	}

	override fun disabledPeriodic() {
		LEDs.currentMode = LEDsMode.ROBOT_DISABLED
	}

	override fun disabledExit() {
		Loader.idleMode = IdleMode.kBrake
		Intake.idleMode = IdleMode.kBrake
		LEDs.currentMode = DEFAULT
	}

	override fun testInit() {
		// Cancels all running commands at the start of test mode.
		commandScheduler.cancelAll()
	}
}