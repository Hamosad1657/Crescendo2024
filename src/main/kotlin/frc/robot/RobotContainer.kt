package frc.robot

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.degrees
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.util.sendable.SendableRegistry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.commands.*
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.swerve.SwerveConstants
import kotlin.math.absoluteValue
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
	const val JOYSTICK_MOVED_THRESHOLD = 0.1

	private val controllerA = CommandPS5Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val controllerB = CommandPS5Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)
	private val testingController = CommandPS5Controller(RobotMap.TESTING_CONTROLLER_PORT)

	val controllerAJoysticksMoving: () -> Boolean = {
		controllerA.leftX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD ||
			controllerA.leftY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD
		controllerA.rightX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD ||
			controllerA.rightY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD
	}

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

	fun sendSubsystemInfo() {
		SmartDashboard.putData(Swerve)
		SmartDashboard.putData(Climbing)
		SmartDashboard.putData(Intake)
		SmartDashboard.putData(Loader)
		SmartDashboard.putData(Shooter)
	}

	fun sendCompetitionInfo() {
		SmartDashboard.putData("Data") { builder ->
			builder.addBooleanProperty("Note detected", Loader::isNoteDetected, null)
			builder.addBooleanProperty("Shooter at setpoint", Shooter::isWithinAngleTolerance, null)
		}
	}

	fun removeSubsystemInfo() {
		SendableRegistry.remove(Swerve)
		SendableRegistry.remove(Climbing)
		SendableRegistry.remove(Intake)
		SendableRegistry.remove(Loader)
		SendableRegistry.remove(Shooter)
	}

	private fun initSendables() {
		if (Robot.telemetryLevel == Telemetry.Competition) {
			sendCompetitionInfo()
			return
		}
		sendSubsystemInfo()
	}

	private fun configureButtonBindings() {
		with(controllerA) {
			// --- Swerve ---
			options().onTrue((Swerve::zeroGyro).asInstantCommand)
			cross().onTrue(Swerve.crossLockWheelsCommand() until controllerAJoysticksMoving)
			PS().toggleOnTrue(Intake.ejectFromIntakeCommand())
			circle().toggleOnTrue(Swerve.driveToTrapCommand().andThen(Notes.loadAndShootCommand(ShooterState.TO_TRAP)))
			square().toggleOnTrue(
				Swerve.aimAtSpeakerWhileDrivingCommand(
					vxSupplier = { controllerA.leftY },
					vySupplier = { controllerA.leftX },
					isFieldRelative = { swerveIsFieldRelative },
				)
			)

			// --- Notes ---
			R1().toggleOnTrue(Loader.loadToShooterOrAmpCommand())
			L1().toggleOnTrue(Notes.collectCommand())
		}

		with(controllerB) {
			square().toggleOnTrue(Shooter.getToShooterStateCommand { ShooterState.TO_TRAP })
			triangle().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.TO_AMP))
			circle().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.AT_SPEAKER))
			cross().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.NEAR_SPEAKER))
			options().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.AT_CLOSER_STAGE) alongWith
				Swerve.getToOneAngleCommand {
					(SwerveConstants.AT_CLOSER_SPEAKER_ANGLE.degrees +
						Swerve.robotHeading.degrees).degrees
				})

			povUp().toggleOnTrue(Climbing.getToOpenedLimitCommand())
			povDown().toggleOnTrue(Climbing.getToClosedLimitCommand())
			R2().toggleOnTrue(Shooter.dynamicShootingCommand())
		}
	}

	private fun setDefaultCommands() {
		Swerve.defaultCommand = Swerve.teleopDriveCommand(
			vxSupplier = { controllerA.leftY },
			vySupplier = { controllerA.leftX },
			omegaSupplier = { controllerA.rightX },
			isFieldRelative = { swerveIsFieldRelative },
			isClosedLoop = { Robot.isAutonomous },
		)

		// Shooter default commands are set in Robot.kt
		Intake.defaultCommand = Intake.run { Intake.stopMotors() }
		Loader.defaultCommand = Loader.run { Loader.stopMotor() }
	}

	fun getAutonomousCommand(): Command {
		return Swerve.followAutoCommand("calibration_auto")
	}

	private fun registerAutoCommands() {
		NamedCommands.registerCommand("eject_command", Notes.loadAndShootCommand(ShooterState.EJECT))
		NamedCommands.registerCommand("collect_command", Notes.autoCollectCommand())
		NamedCommands.registerCommand(
			"shoot_auto_line_1_3_command",
			Notes.loadAndShootCommand(ShooterState.AUTO_LINE_ONE_THREE)
		)
		NamedCommands.registerCommand(
			"shoot_auto_line_2_command",
			Notes.loadAndShootCommand(ShooterState.AUTO_LINE_TWO)
		)
		NamedCommands.registerCommand("shoot_from_speaker_command", Notes.loadAndShootCommand(ShooterState.AT_SPEAKER))

		NamedCommands.registerCommand("shoot_trap_command", Notes.loadAndShootCommand(ShooterState.TO_TRAP))

		NamedCommands.registerCommand("raise_climbing_command", Climbing.getToOpenedLimitCommand())

		NamedCommands.registerCommand(
			"drive_to_speaker_command",
			Swerve.driveToTrapCommand().andThen(Notes.loadAndShootCommand(ShooterState.TO_TRAP))
		)
	}
}