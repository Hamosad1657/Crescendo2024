package frc.robot

import com.hamosad1657.lib.Telemetry
import com.hamosad1657.lib.math.simpleDeadband
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.closedLoopTeleop_shooterAngle
import frc.robot.commands.teleopDriveCommand
import frc.robot.subsystems.shooter.ShooterConstants
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
	const val JOYSTICK_DEADBAND = 0.05

	private val controllerA = CommandPS5Controller(RobotMap.DRIVER_A_CONTROLLER_PORT)
	private val controllerB = CommandPS5Controller(RobotMap.DRIVER_B_CONTROLLER_PORT)
	private val testingController = CommandPS5Controller(RobotMap.TESTING_CONTROLLER_PORT)

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
		if (Robot.telemetryLevel == Telemetry.Competition) return
		SmartDashboard.putData(Climbing)
		SmartDashboard.putData(Intake)
		SmartDashboard.putData(Loader)
		SmartDashboard.putData(Shooter)
	}

	private fun configureButtonBindings() {
		controllerA.options().onTrue(InstantCommand({ Swerve.zeroGyro() }))

		Trigger { Robot.isTeleopEnabled }
			.onTrue(
				Shooter.closedLoopTeleop_shooterAngle(
					{ simpleDeadband(testingController.rightY, JOYSTICK_DEADBAND) },
					ShooterConstants.ANGLE_CLOSED_LOOP_TELEOP_MULTIPLIER,
				)
			)
	}


	private fun setDefaultCommands() {
		Swerve.defaultCommand = Swerve.teleopDriveCommand(
			vxSupplier = { simpleDeadband(controllerA.leftY, JOYSTICK_DEADBAND) },
			vySupplier = { simpleDeadband(controllerA.leftX, JOYSTICK_DEADBAND) },
			omegaSupplier = { simpleDeadband(controllerA.rightX, JOYSTICK_DEADBAND) },
			isFieldRelative = { true },
		)

//		Shooter.defaultCommand = Shooter.prepareShooterForCollectingCommand()
	}

	fun getAutonomousCommand(): Command {
		return Swerve.pathFindToPathCommand("to_speaker")
	}

	private fun registerAutoCommands() {
		NamedCommands.registerCommand("HelloCommand", PrintCommand("HelloWorld"))
	}
}