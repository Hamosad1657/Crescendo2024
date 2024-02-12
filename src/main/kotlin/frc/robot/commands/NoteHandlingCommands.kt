@file:Suppress("FunctionName")

package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.FractionalOutput
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.loader.LoaderConstants
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem

/** - Requirements: intake, loader & shooter. */
fun collectCommand(): Command =
	ShooterSubsystem.prepareShooterForCollectingCommand() alongWith
			collectIntoLoaderCommand() withName "collect"

/** SHOULD BE THE DEFAULT COMMAND OF SHOOTER SUBSYSTEM */
fun ShooterSubsystem.prepareShooterForCollectingCommand(): Command =
	withName("prepare shooter for collecting") { getToShooterStateCommand(ShooterState.COLLECT) }

/** - Requirements: loader & shooter. */
fun loadAndShootCommand(state: ShooterState): Command =
	ShooterSubsystem.getToShooterStateCommand(state) raceWith
			loadIntoShooterCommand() withName "load and shoot"

/**
 * No end condition. This is intentional.
 * - Requirements: shooter.
 */
fun ShooterSubsystem.getToShooterStateCommand(state: ShooterState): Command =
	withName("get to shooter state") {
		run { setShooterState(state) }
	}


// ---
//
// Other than for testing, the commands below should only be used in other command groups in this file.
//
// ---

/**
 * Apart from testing, should only be used in [collectIntoLoaderCommand] or in a manual override.
 *
 * Runs intake only if shooter angle is within tolerance, and loader is running.
 * - Requirements: intake.
 */
fun IntakeSubsystem.runIntakeCommand(): Command =
	withName("run") {
		run {
			if (ShooterSubsystem.isWithinAngleTolerance || LoaderSubsystem.isRunning) {
				set(IntakeConstants.MOTOR_OUTPUT)
			} else {
				set(0.0)
			}
		} finallyDo { _ ->
			set(0.0)
		}
	}


/**
 * Apart from testing, should only be used in [collectIntoLoaderCommand] or [loadIntoShooterCommand], or in a manual override.
 * - Requirements: loader.
 */
fun LoaderSubsystem.runLoaderCommand(): Command =
	withName("run") {
		run {
			set(LoaderConstants.MOTOR_OUTPUT)
		} finallyDo { _ ->
			set(0.0)
		}
	}


/**
 * Apart from testing, should only be used in [collectCommand] or in a manual override.
 * - Requirements: intake & loader.
 */
fun collectIntoLoaderCommand(): Command =
	ShooterSubsystem.prepareShooterForCollectingCommand() alongWith
			LoaderSubsystem.runLoaderCommand() alongWith
			IntakeSubsystem.runIntakeCommand() until
			LoaderSubsystem::isNoteDetected withName "collect into loader"


/**
 * Apart from testing, should only be used in [loadAndShootCommand] or in a manual override.
 * - Requirements: loader & shooter.
 */
fun loadIntoShooterCommand(): Command =
	WaitUntilCommand(ShooterSubsystem::isWithinTolerance) andThen
			LoaderSubsystem.runLoaderCommand() withTimeout
			ShooterConstants.SHOOT_TIME_SEC withName "load into shooter"


// ---
//
// Manual overrides
//
// ---

/** - Requirements: shooter. */
fun ShooterSubsystem.openLoopTeleop_shooterAngle(output: () -> FractionalOutput): Command =
	withName("angle open loop teleop") {
		run {
			setAngleMotorOutput(output())
		} finallyDo { _ ->
			setAngleMotorOutput(0.0)
		}
	}


/**
 * [changeInAngle] is assumed -1 to 1, will come from joysticks.
 * To modify the rate of change, use [multiplier].
 *
 * - Requirements: shooter.
 */
fun ShooterSubsystem.closedLoopTeleop_shooterAngle(changeInAngle: () -> Double, multiplier: Double): Command =
	withName("angle closed loop teleop") {
		run {
			val delta = changeInAngle() * multiplier
			increaseAngleSetpointBy(Rotation2d.fromDegrees(delta))
		}
	}

/** - Requirements: shooter. */
fun ShooterSubsystem.openLoopTeleop_shooterVelocity(output: () -> FractionalOutput): Command =
	withName("velocity open loop teleop") {
		run {
			increaseShooterMotorsOutputBy(output())
		} finallyDo { _ ->
			setShooterMotorsOutput(0.0)
		}
	}


/**
 * [changeInVelocity] is assumed -1 to 1, will come from joysticks.
 * To modify the rate of change, use [multiplier].
 *
 * - Requirements: shooter.
 */
fun ShooterSubsystem.closedLoopTeleop_shooterVelocity(changeInVelocity: () -> Double, multiplier: Double): Command =
	withName("velocity closed loop teleop") {
		run {
			val delta = changeInVelocity() * multiplier
			increaseVelocitySetpointBy(AngularVelocity.fromRpm(delta))
		}
	}

/**
 * Runs the intake in reverse, regardless of shooter angle.
 * - Requirements: intake.
 */
fun IntakeSubsystem.ejectFromIntakeCommand(): Command =
	run {
		set(-IntakeConstants.MOTOR_OUTPUT)
	} finallyDo {
		set(0.0)
	}

/** - Requirements: loader & shooter. */
fun ejectFromShooterCommand(): Command =
	LoaderSubsystem.runLoaderCommand() alongWith
			ShooterSubsystem.run {
				ShooterSubsystem.setShooterMotorsOutput(ShooterConstants.EJECT_OUTPUT)
			} finallyDo {
		ShooterSubsystem.setShooterMotorsOutput(0.0)
	}
