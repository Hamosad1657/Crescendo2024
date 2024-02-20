package frc.robot

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.math.simpleDeadband
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.commands.*
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.climbing.ClimbingSubsystem as Climbing
import frc.robot.subsystems.intake.IntakeSubsystem as Intake
import frc.robot.subsystems.loader.LoaderSubsystem as Loader
import frc.robot.subsystems.shooter.ShooterSubsystem as Shooter
import frc.robot.subsystems.swerve.SwerveSubsystem as Swerve

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
	const val JOYSTICK_DEADBAND = 0.02

	private val controllerA = CommandPS5Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val controllerB = CommandPS5Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)
	private val testingController = CommandPS5Controller(RobotMap.TESTING_CONTROLLER_PORT)

	private var swerveIsFieldRelative = true

	private val autoChooser = Swerve.let {
		AutoBuilder.buildAutoChooser().apply {
			SmartDashboard.putData("Auto Chooser", this)
		}
	}

	init {
		initSendables()
		configureButtonBindings()
		setDefaultCommands()
		registerAutoCommands()
	}

	private fun initSendables() {
		if (Robot.telemetryLevel == Telemetry.Competition) {
			SmartDashboard.putData("Data") { builder ->
				builder.addBooleanProperty("Note", Loader::isNoteDetected, null)
				builder.addBooleanProperty("Intake", Intake::isRunning, null)
			}
			return
		}
		SmartDashboard.putData(Swerve)
		SmartDashboard.putData(Climbing)
		SmartDashboard.putData(Intake)
		SmartDashboard.putData(Loader)
		SmartDashboard.putData(Shooter)
	}

	private fun configureButtonBindings() {
		// --- Swerve ---
		controllerA.options().onTrue(InstantCommand(Swerve::zeroGyro))
		controllerA.PS().onTrue(Swerve.crossLockWheelsCommand())
		controllerA.R2().toggleOnTrue(Loader.loadToShooterOrAmpCommand())
		controllerA.R1().toggleOnTrue(Notes.collectCommand())
		controllerA.create().onTrue(InstantCommand({ swerveIsFieldRelative = !swerveIsFieldRelative }))
		controllerA.square().onTrue(InstantCommand({}, Swerve))
		// --- Notes ---
		// # Controller A #
		controllerB.square().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.AT_STAGE))
		controllerB.triangle().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.TO_AMP))
		controllerB.circle().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.AT_SPEAKER))
		controllerB.cross().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.TO_TRAP))
	}

	private fun setDefaultCommands() {
		Swerve.defaultCommand = Swerve.teleopDriveCommand(
			vxSupplier = { controllerA.leftY },
			vySupplier = { controllerA.leftX },
			omegaSupplier = { controllerA.rightX },
			isFieldRelative = { swerveIsFieldRelative },
			isClosedLoop = { true },
		)

		Shooter.defaultCommand = Shooter.getToShooterStateCommand(ShooterState.COLLECT)
		Climbing.defaultCommand =
			Climbing.openLoopTeleopCommand { simpleDeadband(controllerB.rightY, JOYSTICK_DEADBAND) }


	}

	fun getAutonomousCommand(): Command {
		return Swerve.followAutoCommand("three_part_auto")
	}

	private fun registerAutoCommands() {
		NamedCommands.registerCommand("eject_command", Notes.loadAndShootCommand(ShooterState.EJECT))
		NamedCommands.registerCommand("collect_command", Notes.collectCommand())

	}
}