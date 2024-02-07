package frc.robot

import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.math.simpleDeadband
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.commands.openLoopTeleop_shooterAngle
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.subsystems.arm.ArmSubsystem
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
	private val controllerA = CommandPS5Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val controllerB = CommandPS5Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)
	private val testingController = CommandPS5Controller(RobotMap.TESTING_CONTROLLER_PORT)
	private val swerve = SwerveSubsystem
	private val autoChooser = AutoBuilder.buildAutoChooser().apply { SmartDashboard.putData("Auto Chooser", this) }

	init {
		registerAutoCommands()
		configureBindings()
		setDefaultCommands()
	}

	/** Use this method to define your `trigger->command` mappings. */
	private fun configureBindings() {
		autoChooser.onChange { controllerA.triangle().onTrue(it) }
		controllerA.options().onTrue(InstantCommand({ swerve.zeroGyro() }))
		controllerA.square().onTrue(InstantCommand({}, swerve))
	}

	private fun setDefaultCommands() {
		swerve.defaultCommand = TeleopDriveCommand(
			swerve,
			vX = { simpleDeadband(controllerA.leftY, 0.1) },
			vY = { simpleDeadband(controllerA.leftX, 0.1) },
			omega = { simpleDeadband(controllerA.rightX * 1.0, 0.1) },
			isFieldRelative = { true },
			headingCorrection = false
		)

		// --- For initial testing, delete later --- //

		// Test 1 thing at a time.
		// For a list of things to test follow the link:
		// https://docs.google.com/document/d/1App5L-vltuqvOiloeHfqbKvk7FwQHXPcqmUYKuAhA1A/edit
		
		with(ShooterSubsystem) {
			defaultCommand = openLoopTeleop_shooterAngle { testingController.leftY }
//			defaultCommand = openLoopTeleop_shooterVelocity { testingController.leftY }
		}

//		with(IntakeSubsystem) {
//			defaultCommand = run {
//				set(IntakeConstants.MOTOR_OUTPUT)
//			} finallyDo {
//				set(0.0)
//			}
//		}

//		with(LoaderSubsystem) {
//			defaultCommand = run {
//				set(LoaderConstants.MOTOR_OUTPUT)
//			} finallyDo {
//				set(0.0)
//			}
//		}
	}

	private fun registerAutoCommands() {
		NamedCommands.registerCommand("HelloCommand", PrintCommand("HelloWorld"))
	}

	fun getAutonomousCommand(): Command {
		// TODO: Implement properly
		return swerve.pathFindToPathCommand("example_path")
	}

	fun setAllMechanismsToCoast() {
		ShooterSubsystem.angleNeutralMode = NeutralModeValue.Coast
		ShooterSubsystem.shooterNeutralMode = NeutralModeValue.Coast
		LoaderSubsystem.neutralMode = NeutralModeValue.Coast
		IntakeSubsystem.neutralMode = NeutralModeValue.Coast
		ArmSubsystem.neutralMode = NeutralModeValue.Coast
		ClimbingSubsystem.neutralMode = NeutralModeValue.Coast
	}

	fun setAllMechanismsNeutralMode() {
		ShooterSubsystem.angleNeutralMode = NeutralModeValue.Brake
		ShooterSubsystem.shooterNeutralMode = NeutralModeValue.Coast
		LoaderSubsystem.neutralMode = NeutralModeValue.Brake
		IntakeSubsystem.neutralMode = NeutralModeValue.Brake
		ArmSubsystem.neutralMode = NeutralModeValue.Brake
		ClimbingSubsystem.neutralMode = NeutralModeValue.Brake
	}
}