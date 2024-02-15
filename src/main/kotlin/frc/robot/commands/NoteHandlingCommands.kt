package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.PercentOutput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.loader.LoaderConstants
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.intake.IntakeSubsystem as Intake
import frc.robot.subsystems.loader.LoaderSubsystem as Loader
import frc.robot.subsystems.shooter.ShooterSubsystem as Shooter

/** - Requirements: Intake, Loader, Shooter. */
fun collectCommand(): Command = withName("collect") {
	(Shooter.prepareShooterForCollectingCommand() alongWith
		Loader.runLoaderCommand() alongWith
		Intake.runIntakeCommand()
		) until
		Loader::isNoteDetected
}

/** SHOULD BE THE DEFAULT COMMAND OF SHOOTER SUBSYSTEM */
fun Shooter.prepareShooterForCollectingCommand(): Command = withName("prepare shooter for collecting") {
	getToShooterStateCommand(ShooterState.COLLECT)
}

/** - Requirements: Loader, Shooter. */
fun loadAndShootCommand(state: ShooterState): Command = withName("load and shoot") {
	Shooter.getToShooterStateCommand(state) raceWith
		(waitUntil { Shooter.isWithinTolerance } andThen
			loadIntoShooterCommand())
}

/**
 * - Command has no end condition.
 * - Requirements: Shooter.
 */
fun Shooter.getToShooterStateCommand(state: ShooterState): Command = withName("get to shooter state") {
	run {
		setShooterState(state)
	} finallyDo {
		stopShooterMotors()
	}
}


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
fun waitForNoteToPassCommand() = withName("wait for note to pass") {
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
 * - Requirements: Intake.
 */
fun Intake.runIntakeCommand(): Command = withName("run") {
	run {
		if (Shooter.isWithinAngleTolerance || Loader.isRunning) {
			set(IntakeConstants.BOTTOM_MOTOR_OUTPUT, IntakeConstants.TOP_MOTOR_OUTPUT)
		} else {
			stop()
		}
	} finallyDo {
		stop()
	}
}


/**
 * Apart from testing, should only be used in [collectCommand] or [loadIntoShooterCommand], or in a manual override.
 * - Requirements: Loader.
 */
fun Loader.runLoaderCommand(): Command = withName("run") {
	run {
		set(LoaderConstants.MOTOR_OUTPUT)
	} finallyDo {
		stop()
	}
}

/**
 * Apart from testing, should only be used in [loadAndShootCommand] or in a manual override.
 * - Requirements: Loader.
 */
fun loadIntoShooterCommand(): Command = withName("load into shooter") {
	Loader.runLoaderCommand() withTimeout
		ShooterConstants.SHOOT_TIME_SEC
}


// ---
//
// Manual overrides
//
// ---

/** - Requirements: Shooter. */
fun Shooter.openLoopTeleop_shooterVelocity(
	output: () -> PercentOutput
): Command = withName("velocity open loop teleop") {
	run {
		increaseShooterMotorsOutputBy(output())
	} finallyDo {
		stopShooterMotors()
	}
}


/**
 * [changeInVelocity] is assumed -1 to 1, will come from joysticks.
 * To modify the rate of change, use [multiplier].
 *
 * - Requirements: Shooter.
 */
fun Shooter.closedLoopTeleop_shooterVelocity(
	changeInVelocity: () -> Double, multiplier: Double
): Command = withName("velocity closed loop teleop") {
	run {
		val delta = changeInVelocity() * multiplier
		increaseVelocitySetpointBy(AngularVelocity.fromRpm(delta))
	}
}

/**
 * Runs the intake in reverse, regardless of shooter angle.
 * - Requirements: Intake.
 */
fun Intake.ejectFromIntakeCommand(): Command =
	run {
		set(-IntakeConstants.BOTTOM_MOTOR_OUTPUT, -IntakeConstants.TOP_MOTOR_OUTPUT)
	} finallyDo {
		stop()
	}

/** - Requirements: Loader, Shooter. */
fun ejectFromShooterCommand(): Command =
	Loader.runLoaderCommand() alongWith
		Shooter.run {
			Shooter.setShooterMotorsOutput(ShooterConstants.EJECT_OUTPUT)
		}.finallyDo { _ ->
			Shooter.stopShooterMotors()
		}
