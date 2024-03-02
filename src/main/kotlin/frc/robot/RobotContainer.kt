package frc.robot

//import frc.robot.subsystems.climbing.ClimbingSubsystem as Climbing
import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.robotPrint
import com.hamosad1657.lib.units.degrees
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.commands.*
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import kotlin.jvm.optionals.getOrNull
import kotlin.math.absoluteValue
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

	//	const val CLIMBING_DEADBAND = 0.08
	const val JOYSTICK_MOVED_THRESHOLD = 0.1

	private val controllerA = CommandPS5Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val controllerB = CommandPS5Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)
	private val testingController = CommandPS5Controller(RobotMap.TESTING_CONTROLLER_PORT)

	private var swerveTeleopMultiplier = 1.0

	val controllerAJoysticksMoving: () -> Boolean = {
		(controllerA.leftX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
			(controllerA.leftY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
			(controllerA.rightX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
			(controllerA.rightY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD)
	}

	val controllerBJoysticksMoving: () -> Boolean = {
		(controllerB.leftY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
			(controllerB.leftX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
			(controllerB.rightY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
			(controllerB.rightX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD)
	}

	private var swerveIsFieldRelative = true

	init {
		SwerveSubsystem
		registerAutoCommands()
	}

	private val autoChooser =
		AutoBuilder.buildAutoChooser("shoot_one").apply {
			onChange {
				robotPrint(it.name)
			}
		}

	private val allianceChooser =
		SendableChooser<Alliance>().apply {
			setDefaultOption("Blue", Alliance.Blue)
			addOption("Red", Alliance.Red)

			DriverStation.getAlliance().getOrNull()?.let {
				Robot.alliance = it
				setDefaultOption(it.name, it)
			}

			onChange {
				Robot.alliance = it
				robotPrint(it.name)
			}
		}

	init {
		configureButtonBindings()
		initSendables()
		setDefaultCommands()
	}

	fun sendSubsystemInfo() {
		SmartDashboard.putData(Swerve)
//		SmartDashboard.putData(Climbing)
		SmartDashboard.putData(Intake)
		SmartDashboard.putData(Loader)
		SmartDashboard.putData(Shooter)
	}

	fun sendCompetitionInfo() {
		with(Shuffleboard.getTab("Auto")) {
			add("Auto chooser", autoChooser).withSize(3, 1).withPosition(2, 1)
			add("Alliance", allianceChooser).withSize(3, 1).withPosition(7, 1)
		}

		with(Shuffleboard.getTab("Driving")) {
			addBoolean("Note detected", Loader::isNoteDetected).withSize(3, 1).withPosition(2, 1)
			addBoolean("Shooter at setpoint", Shooter::isWithinAngleTolerance).withSize(3, 1).withPosition(2, 3)
			addBoolean("Intake running", Intake::isRunning).withSize(3, 1).withPosition(7, 1)
//			addBoolean("Left TRAP switch pressed", Climbing::isLeftTrapSwitchPressed).withPosition(6, 1).withSize(2, 1)
//			addBoolean("Right TRAP switch pressed", Climbing::isRightTrapSwitchPressed).withPosition(8, 1).withSize(2, 1)
		}
	}

	private fun initSendables() {
		sendCompetitionInfo()
		if (Robot.telemetryLevel == Telemetry.Testing) {
			sendSubsystemInfo()
		}
	}

	private fun configureButtonBindings() {
		with(controllerA) {
			// --- Swerve ---
			options().onTrue((Swerve::zeroGyro).asInstantCommand)
			cross().onTrue(Swerve.crossLockWheelsCommand() until controllerAJoysticksMoving)
			PS().toggleOnTrue(Intake.ejectFromIntakeCommand())
			circle().toggleOnTrue(
				(Swerve.driveToTrapCommand() raceWith
					Shooter.getToShooterStateCommand(ShooterState.TO_TRAP)) andThen
					Notes.loadAndShootCommand(ShooterState.TO_TRAP)
			)
			square().toggleOnTrue(Swerve.getToOneAngleCommand {
				(SwerveConstants.AT_PODIUM_ANGLE.degrees +
					Swerve.robotHeading.degrees).degrees
			} until controllerAJoysticksMoving)
			povDown().onTrue({ swerveTeleopMultiplier = 0.5 }.asInstantCommand)
			povUp().onTrue({ swerveTeleopMultiplier = 1.0 }.asInstantCommand)

			povRight().onTrue(InstantCommand({ Swerve.resetOdometry(Pose2d(1.0, 1.0, 120.degrees)) }))

			// --- Notes ---
			R1().toggleOnTrue(Loader.loadToShooterOrAmpCommand())
			L1().toggleOnTrue(Notes.collectCommand())
			create().toggleOnTrue(Notes.collectFromHumanPlayerCommand())
		}

		with(controllerB) {
			square().toggleOnTrue(Shooter.getToShooterStateCommand { ShooterState.TO_TRAP })
			triangle().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.TO_AMP))
			circle().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.AT_SPEAKER))
			cross().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.NEAR_SPEAKER))
			options().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.AT_PODIUM))

//			povUp().toggleOnTrue(Climbing.getToOpenedLimitCommand().until(controllerBJoysticksMoving))
//			povDown().toggleOnTrue(Climbing.getToClosedLimitCommand().until(controllerBJoysticksMoving))
			// R2().toggleOnTrue(Shooter.dynamicShootingCommand())
		}
	}

	private fun setDefaultCommands() {
		Swerve.defaultCommand = Swerve.teleopDriveCommand(
			vxSupplier = { controllerA.leftY * swerveTeleopMultiplier },
			vySupplier = { controllerA.leftX * swerveTeleopMultiplier },
			omegaSupplier = { controllerA.rightX * swerveTeleopMultiplier },
			isFieldRelative = { swerveIsFieldRelative },
			isClosedLoop = { Robot.isAutonomous },
		)

		// Shooter default commands are set in Robot.kt
		Intake.defaultCommand = Intake.run { Intake.stopMotors() }
		Loader.defaultCommand = Loader.run { Loader.stopMotor() }

//		Climbing.defaultCommand =
//			Climbing.brokenOpenLoopTeleopCommand(
//				{ simpleDeadband(controllerB.leftY, CLIMBING_DEADBAND) },
//				{ simpleDeadband(controllerB.rightY, CLIMBING_DEADBAND) })
	}

	fun getAutonomousCommand(): Command {
		return autoChooser.selected
//		return Swerve.followAutoCommand("shoot_and_move_away")
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

//		NamedCommands.registerCommand("raise_climbing_command", Climbing.getToOpenedLimitCommand())

		NamedCommands.registerCommand(
			"drive_to_speaker_command",
			Swerve.driveToTrapCommand().andThen(Notes.loadAndShootCommand(ShooterState.TO_TRAP))
		)
	}
}