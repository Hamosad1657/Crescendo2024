package frc.robot

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.robotPrint
import com.hamosad1657.lib.units.plus
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import frc.robot.commands.*
import frc.robot.subsystems.shooter.DynamicShooting
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.vision.NoteVision
import java.util.Optional
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.sign
import frc.robot.subsystems.intake.IntakeSubsystem as Intake
import frc.robot.subsystems.loader.LoaderSubsystem as Loader
import frc.robot.subsystems.shooter.ShooterSubsystem as Shooter
import frc.robot.subsystems.swerve.SwerveSubsystem as Swerve

fun joystickCurve(value: Double) = -value.pow(2) * value.sign

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
	const val JOYSTICK_DEADBAND = 0.02
	//	const val CLIMBING_DEADBAND = 0.08

	private val controllerA = CommandPS5Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val controllerB = CommandPS5Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)
	private val testingController = CommandPS5Controller(RobotMap.TESTING_CONTROLLER_PORT)

	private var swerveTeleopMultiplier = 1.0
	private var swerveIsFieldRelative = true

	init {
		SwerveSubsystem
		registerAutoCommands()
		configureButtonBindings()
		setDefaultCommands()
	}


	// --- Robot Operation ---

	private fun configureButtonBindings() {
		with(controllerA) {
			// --- Swerve ---
			// Zero gyro
			options().onTrue((Swerve::zeroGyro).asInstantCommand)

			// Lock wheels
			cross().onTrue(Swerve.crossLockWheelsCommand() until ::areControllerAJoysticksMoving)

			// Speed controls
			povDown().onTrue({ swerveTeleopMultiplier = 0.5 }.asInstantCommand)
			povUp().onTrue({ swerveTeleopMultiplier = 1.0 }.asInstantCommand)

			// Rotate to speaker at podium
			circle().toggleOnTrue(
				Swerve.getToOneAngleCommand {
					SwerveConstants.AT_PODIUM_TO_SPEAKER_ROTATION plus Swerve.robotHeading
				} until ::areControllerAJoysticksMoving
			)

			// Rotate to speaker at stage
			triangle().toggleOnTrue(
				Swerve.getToOneAngleCommand {
					SwerveConstants.AT_STAGE_TO_SPEAKER_ROTATION plus Swerve.robotHeading
				} until ::areControllerAJoysticksMoving
			)


			// --- Notes ---
			// Dynamic shooting
			square().whileTrue(Swerve.aimAtSpeakerWhileDrivingCommand(
				vxSupplier = { controllerA.leftY * swerveTeleopMultiplier },
				vySupplier = { controllerA.leftX * swerveTeleopMultiplier }
			) alongWith Shooter.dynamicShootingCommand())

			// Collect
			L1().toggleOnTrue(
				Notes.collectCommand() raceWith
					SwerveSubsystem.aimAtNoteWhileDrivingCommand(
						vxSupplier = { controllerA.leftY * swerveTeleopMultiplier },
						vySupplier = { controllerA.leftX * swerveTeleopMultiplier },
						omegaSupplier = { controllerA.rightX }
					)
			)

			// Collect from human player
			create().toggleOnTrue(Notes.collectFromHumanPlayerCommand())

			// Load
			R1().toggleOnTrue(Loader.loadToShooterOrAmpCommand())

			// Eject
			PS().toggleOnTrue(Intake.ejectFromIntakeCommand())
		}

		with(controllerB) {
			// Speaker
			circle().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.AT_SPEAKER))
			cross().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.NEAR_SPEAKER))
			options().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.AT_PODIUM))
			create().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.AT_STAGE))

			R2().toggleOnTrue(Shooter.dynamicShootingCommand())

			// Amp & Trap
			triangle().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.TO_AMP))
			square().toggleOnTrue(Shooter.getToShooterStateCommand(ShooterState.TO_TRAP))

			// Climbing
//			povUp().toggleOnTrue(Climbing.getToOpenedLimitCommand().until(controllerBJoysticksMoving))
//			povDown().toggleOnTrue(Climbing.getToClosedLimitCommand().until(controllerBJoysticksMoving))
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
		Loader.defaultCommand = Loader.run { Loader.stopMotors() }

//		Climbing.defaultCommand =
//			Climbing.brokenOpenLoopTeleopCommand(
//				{ simpleDeadband(controllerB.leftY, CLIMBING_DEADBAND) },
//				{ simpleDeadband(controllerB.rightY, CLIMBING_DEADBAND) })
	}


	// --- Choosers ---

	private val autoChooser =
		AutoBuilder.buildAutoChooser("shoot_one").apply {
			onChange {
				robotPrint(it.name)
			}
		}

	private val allianceChooser =
		SendableChooser<DriverStation.Alliance>().apply {
			setDefaultOption("Blue", DriverStation.Alliance.Blue)
			addOption("Red", DriverStation.Alliance.Red)

			onChange {
				Robot.alliance = it
				robotPrint(it.name)
			}
		}

	init {
		initSendables()
	}


	// --- Telemetry ---

	private fun initSendables() {
		sendCompetitionInfo()
		if (Robot.isTesting) sendSubsystemInfo()
	}

	private fun sendSubsystemInfo() {
		SmartDashboard.putData(Swerve)
		SmartDashboard.putData(Intake)
		SmartDashboard.putData(Loader)
		SmartDashboard.putData(Shooter)
//		SmartDashboard.putData(Climbing)
	}

	private fun sendCompetitionInfo() {
		with(Shuffleboard.getTab("Auto")) {
			add("Auto chooser", autoChooser).withSize(3, 1).withPosition(2, 1)
			addString("Submitted Auto") {
				Robot.submittedAuto?.name ?: "None submitted, will use currently selected in Auto Chooser"
			}
				.withSize(3, 1)
				.withPosition(2, 3)
			add("Submit Auto", InstantCommand(::submitAuto)).withSize(3, 1).withPosition(7, 3)
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

	// --- Auto ---

	private fun submitAuto() {
		Robot.submittedAuto = autoChooser.selected
		robotPrint("Submitted auto")
	}

	fun getAutonomousCommand(): Command = Robot.submittedAuto ?: autoChooser.selected

	private fun registerAutoCommands() {
		fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)

		register("eject_command", Notes.loadAndShootCommand(ShooterState.EJECT))
		register(
			"collect_command", Notes.autoCollectCommand().beforeStarting(
			InstantCommand()
		)
		)
		register("collect_with_timeout_command", Notes.autoCollectCommand() withTimeout 2.0
			andThen InstantCommand({
			PPHolonomicDriveController.setRotationTargetOverride { Optional.empty() }
		}
		))
		register("shoot_command",
			Notes.loadAndShootCommand {
				DynamicShooting.calculateShooterState(Swerve.robotPose.translation)
			}
		)
		register("shoot_auto_line_1_3_command", Notes.loadAndShootCommand(ShooterState.AUTO_LINE_ONE_THREE))
		register("shoot_auto_line_2_command", Notes.loadAndShootCommand(ShooterState.AUTO_LINE_TWO))
		register("shoot_from_speaker_command", Notes.loadAndShootCommand(ShooterState.AT_SPEAKER))
		register("aim_at_note_command", InstantCommand({
			PPHolonomicDriveController.setRotationTargetOverride {
				val rotationTarget = NoteVision.getRobotToBestTargetYawDelta()?.let {
					Optional.of(SwerveSubsystem.robotHeading plus it)
				}
				rotationTarget ?: Optional.empty()
			}
		}))
	}


	// --- Joysticks ---

	private const val JOYSTICK_MOVED_THRESHOLD = 0.1

	private val areControllerAJoysticksMoving
		get() =
			(controllerA.leftX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
				(controllerA.leftY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
				(controllerA.rightX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
				(controllerA.rightY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD)


	private val areControllerBJoysticksMoving
		get() =
			(controllerB.leftY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
				(controllerB.leftX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
				(controllerB.rightY.absoluteValue >= JOYSTICK_MOVED_THRESHOLD) or
				(controllerB.rightX.absoluteValue >= JOYSTICK_MOVED_THRESHOLD)
}