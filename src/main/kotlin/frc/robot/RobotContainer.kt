package frc.robot

import com.hamosad1657.lib.Telemetry
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.commands.*
import frc.robot.subsystems.loader.LoaderConstants
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
		controllerA.create().onTrue(InstantCommand({ swerveIsFieldRelative = !swerveIsFieldRelative }))
		controllerA.square().onTrue(InstantCommand({}, Swerve))
		controllerA.PS().onTrue(InstantCommand({
			Swerve.resetOdometry(
				Pose2d()
			)
		}))

		// --- Notes ---
		// # Controller A #
		controllerA.R1().toggleOnTrue(Notes.collectCommand())
		controllerA.R2().whileTrue(Loader.runLoaderCommand(LoaderConstants.MOTOR_LOADING_VOLTAGE))
		controllerA.square().toggleOnTrue(
			Notes.loadAndShootCommand(ShooterState.AT_STAGE)
		)
		controllerA.circle().toggleOnTrue(Shooter.getToAngleCommand(ShooterState.AT_STAGE.angle))
		controllerA.L1().toggleOnTrue(Notes.collectCommand(ShooterState.COLLECT_TO_TRAP))
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

//		Climbing.defaultCommand = Climbing.getToOpenLimitCommand()
	}

	fun getAutonomousCommand(): Command {
		return Swerve.followAutoCommand("calibration_auto")
	}

	private fun registerAutoCommands() {
		NamedCommands.registerCommand("HelloCommand", PrintCommand("HelloWorld"))
	}
}