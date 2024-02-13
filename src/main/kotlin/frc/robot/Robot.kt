package frc.robot

import com.hamosad1657.lib.Telemetry
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler

/**
 * The VM is configured to automatically run this object (which basically functions as a singleton class),
 * and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 *
 * If you change the name of this object or its package after creating this project, you must also update
 * the `Main.kt` file in the project. (If you use the IDE's Rename or Move refactorings when renaming the
 * object or package, it will get changed everywhere.)
 */
object Robot : TimedRobot() {
	var robotTelemetry = Telemetry.Testing.also { SmartDashboard.putString("Telemetry", it.name) }
		set(value) {
			SmartDashboard.putString("Telemetry", field.name)
			field = value
		}

	private var autonomousCommand: Command? = null
	private var commandScheduler = CommandScheduler.getInstance()

	fun <T> debugPrint(value: T) = value.also {
		DriverStation.reportWarning(it.toString(), false)
	}

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
		autonomousCommand = RobotContainer.getAutonomousCommand()
		autonomousCommand?.schedule()
	}

	override fun teleopInit() {
		autonomousCommand?.cancel()
	}

	override fun testInit() {
		// Cancels all running commands at the start of test mode.
		commandScheduler.cancelAll()
	}

	override fun simulationInit() {
		robotTelemetry = Telemetry.Simulation
	}

	// TODO: Check if this is the right delay.
	const val DISABLED_COAST_DELAY_SECONDS = 5.0
}