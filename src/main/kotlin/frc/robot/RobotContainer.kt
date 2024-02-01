package frc.robot

import com.hamosad1657.lib.math.simpleDeadband
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
	private val controller = CommandPS5Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val swerve = SwerveSubsystem
	private val autoChooser = AutoBuilder.buildAutoChooser().apply { SmartDashboard.putData("Auto Chooser", this) }

	init {
		registerAutoCommands()
		configureBindings()
		setDefaultCommands()
	}

	/** Use this method to define your `trigger->command` mappings. */
	private fun configureBindings() {
		autoChooser.onChange { controller.triangle().onTrue(it) }
		controller.options().onTrue(InstantCommand({ swerve.zeroGyro() }))
		controller.square().onTrue(InstantCommand({}, swerve))
	}

	private fun setDefaultCommands() {
		swerve.defaultCommand = TeleopDriveCommand(
			swerve,
			vX = { simpleDeadband(controller.leftY, 0.1) },
			vY = { simpleDeadband(controller.leftX, 0.1) },
			omega = { simpleDeadband(controller.rightX * 1.0, 0.1) },
			isFieldRelative = { true },
			headingCorrection = false
		)
	}

	private fun registerAutoCommands() {
		NamedCommands.registerCommand("HelloCommand", PrintCommand("HelloWorld"))
	}

	fun getAutonomousCommand(): Command {
		// TODO: Implement properly
		return swerve.pathFindToPathCommand("example_path")
	}
}