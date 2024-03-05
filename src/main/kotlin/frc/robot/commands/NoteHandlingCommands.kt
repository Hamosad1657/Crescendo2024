@file:Suppress("UnusedReceiverParameter")

package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Volts
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.loader.LoaderConstants
import frc.robot.subsystems.shooter.DynamicShooting
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem as Intake
import frc.robot.subsystems.loader.LoaderSubsystem as Loader
import frc.robot.subsystems.shooter.ShooterSubsystem as Shooter

object Notes

/** - Requirements: Intake, Loader, Shooter. */
fun Notes.collectCommand(shooterState: ShooterState = ShooterState.COLLECT): Command = withName("collect") {
	((Shooter.getToShooterStateCommand(shooterState) alongWith
		Loader.runLoaderCommand(LoaderConstants.MOTOR_INTAKE_OUTPUT) alongWith
		Intake.runIntakeCommand()) until Loader::isNoteDetected
		).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
}

/**
 * Like [collectCommand], but interruptable, and the shooter spins continuously.
 * - Requirements: Intake, Loader, Shooter.
 */
fun Notes.autoCollectCommand(shooterState: ShooterState = ShooterState.AUTO_COLLECT): Command = withName("collect") {
	((Shooter.getToShooterStateCommand(shooterState) alongWith
		Loader.runLoaderCommand(LoaderConstants.MOTOR_INTAKE_OUTPUT) alongWith
		Intake.runIntakeCommand()) until Loader::isNoteDetected
		).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
}

/** - Requirements: Loader, Shooter. */
fun Notes.loadAndShootCommand(state: ShooterState): Command = withName("load and shoot") {
	(Shooter.getToShooterStateCommand(state) raceWith
		(WaitCommand(0.2) andThen
			waitUntil { Shooter.isWithinTolerance }
				.withTimeout(ShooterConstants.SHOOT_TIMEOUT_SEC) andThen
			WaitCommand(0.1) andThen
			loadIntoShooterCommand()))
}

/** - Requirements: Loader, Shooter. */
fun Notes.loadAndShootCommand(stateSupplier: () -> ShooterState): Command = withName("load and shoot") {
	(Shooter.getToShooterStateCommand(stateSupplier) raceWith
		(WaitCommand(0.2) andThen
			waitUntil { Shooter.isWithinTolerance }
				.withTimeout(ShooterConstants.SHOOT_TIMEOUT_SEC) andThen
			WaitCommand(0.1) andThen
			loadIntoShooterCommand()))
}

/**
 * - Command has no end condition.
 * - Requirements: Shooter.
 */
fun Shooter.getToShooterStateCommand(state: ShooterState): Command = withName("get to shooter state") {
	runOnce {
		resetVelocityPIDController()
	} andThen run {
		setShooterState(state)
	} finallyDo {
		stopShooterMotors()
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Shooter.
 */
fun Shooter.getToShooterStateCommand(state: () -> ShooterState): Command = withName("get to shooter state") {
	runOnce {
		resetVelocityPIDController()
	} andThen run {
		setShooterState(state())
	} finallyDo {
		stopShooterMotors()
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Shooter.
 */
fun Shooter.getToAngleCommand(angle: Rotation2d): Command = withName("get to shooter state") {
	run {
		setAngle(angle)
	} finallyDo {
		stopShooterMotors()
	}
}

/** - Requirements: Shooter. */
fun Shooter.dynamicShootingCommand() = Shooter.getToShooterStateCommand {
	SwerveSubsystem.robotPose.let { estimatedPose ->
		DynamicShooting.calculateShooterState(estimatedPose.translation)
	}
}

/**
 * Waits until shooter is within angle tolerance to AMP,
 * then ejects the note through the loader for [ShooterConstants.SHOOT_TIME_SEC] seconds.
 * Finally, interrupts shooter subsystem so it goes back to it's default command.
 * - Requirements: Loader (until the command ends, then Shooter).
 */
fun Loader.ejectIntoAmpCommand(): Command = withName("eject") {
	waitUntil(Shooter::isWithinAngleToleranceToAmp) andThen
		Loader.runLoaderCommand(LoaderConstants.MOTOR_EJECT_OUTPUT) withTimeout
		LoaderConstants.AMP_EJECT_TIME_SEC finallyDo
		Shooter.runOnce {}
}

/**
 * Runs the loader motor for [ShooterConstants.SHOOT_TIME_SEC] seconds.
 * Finally, interrupts shooter subsystem so it goes back to it's default command.
 * - Requirements: Loader (until the command ends, then Shooter.)
 */
fun Loader.loadIntoShooterCommand(): Command = withName("load into shooter") {
	runLoaderCommand(LoaderConstants.MOTOR_LOADING_OUTPUT) withTimeout
		ShooterConstants.SHOOT_TIME_SEC finallyDo
		Shooter.runOnce {}
}

/**
 * Schedules [loadIntoShooterCommand] or [ejectIntoAmpCommand] based on the angle of the shooter.
 * Interrupts shooter subsystem when the scheduled command ends.
 * - Requirements: Loader (until the scheduled command ends, then Shooter).
 */
fun Loader.loadToShooterOrAmpCommand(): Command =
	ConditionalCommand(
		ejectIntoAmpCommand(), // Command on true
		loadIntoShooterCommand(), // Command on false
		Shooter::isWithinAngleToleranceToAmp,
	)


// ---
//
// Other than for testing, the commands below should only be used in other command groups in this file.
//
// ---

/**
 * Apart from testing, should only be used in [collectCommand].
 *
 * Waits for a note to pass the beam-breaker (get detected and then not).
 * - Requirements: None.
 */
fun Notes.waitForNoteToPassCommand() = withName("wait for note to pass") {
	var hasNotePassed = false
	Commands.run({
		if (Loader.isNoteDetected && !hasNotePassed) hasNotePassed = true
	}) until {
		hasNotePassed && !Loader.isNoteDetected
	}
}

/**
 * Apart from testing, should only be used in [collectCommand] or in a manual override.
 *
 * Runs intake only if shooter angle is within tolerance, and loader is running.
 * - Command has no end condition.
 * - Requirements: Intake.
 */
fun Intake.runIntakeCommand(): Command = withName("run") {
	run {
		if ((Shooter.isWithinAngleTolerance || Loader.isRunning) && !Loader.isNoteDetected) {
			setVoltage(IntakeConstants.BOTTOM_MOTOR_OUTPUT, IntakeConstants.TOP_MOTOR_OUTPUT)
		} else {
			stopMotors()
		}
	} finallyDo {
		stopMotors()
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Loader.
 */
fun Loader.runLoaderCommand(voltage: Volts): Command = withName("run") {
	run {
		setVoltage(voltage)
	} finallyDo {
		stopMotors()
	}
}

/** - Requirements: Loader. */
fun Notes.loadIntoShooterCommand(): Command = withName("load into shooter") {
	Loader.runLoaderCommand(LoaderConstants.MOTOR_LOADING_OUTPUT) withTimeout
		ShooterConstants.SHOOT_TIME_SEC
}

/** - Requirements: Intake, Loader, Shooter. */
fun Notes.collectAndEject(): Command = withName("collect and eject") {
	Intake.runIntakeCommand() alongWith
		Loader.runLoaderCommand(LoaderConstants.MOTOR_LOADING_OUTPUT) alongWith
		Shooter.getToShooterStateCommand(ShooterState.EJECT)
}

/** - Requirements: Loader, Shooter. */
fun Notes.collectFromHumanPlayerCommand(): Command = withName("collect from human player") {
	(Shooter.getToShooterStateCommand(ShooterState.COLLECT_FROM_FEEDER) alongWith
		Loader.runLoaderCommand(LoaderConstants.MOTOR_INTAKE_OUTPUT)) until
		Loader::isNoteDetected
}

/**
 * Runs the intake in reverse, regardless of shooter angle.
 * - Requirements: Intake.
 */
fun Intake.ejectFromIntakeCommand(): Command = withName("eject from intake") {
	run {
		setVoltage(-IntakeConstants.BOTTOM_MOTOR_OUTPUT, -IntakeConstants.TOP_MOTOR_OUTPUT)
	} withTimeout IntakeConstants.EJECT_TIME_SEC finallyDo {
		stopMotors()
	}
}
