package frc.robot.commands

import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.until
import com.hamosad1657.lib.commands.withName
import com.hamosad1657.lib.units.Rotations
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants.ClimbingState.*
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

/**
 * Climbing mechanism extends open-loop until it passed [REACHING_CHAIN.setpoint], then the command ends.
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.reachChainCommand(): Command =
	withName("reach chain") {
		runOnce {
			configPIDF(Constants.NO_WEIGHT_BEARING_PID_GAINS)
		} andThen openLoopGetToPositionCommand(REACHING_CHAIN.setpoint, REACHING_CHAIN.output)
	}

/**
 * Climbing mechanism retracts open-loop until it passed [PULLING_UP_ROBOT.setpoint],
 * then performs PID control to maintain that setpoint. Command has no end condition.
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.pullUpRobotCommand(): Command =
	withName("pull up robot") {
		runOnce {
			configPIDF(Constants.WEIGHT_BEARING_PID_GAINS)
		} andThen openLoopGetToPositionCommand(PULLING_UP_ROBOT.setpoint, PULLING_UP_ROBOT.output) andThen
				maintainSetpointCommand(PULLING_UP_ROBOT.setpoint)
	}

/**
 * Climbing mechanism extends open-loop until it passed [CLIMBING_DOWN.setpoint],
 * then performs PID to maintain that setpoint. Command has no end condition.
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.climbDownCommand(): Command =
	withName("climb down") {
		runOnce {
			configPIDF(Constants.NO_WEIGHT_BEARING_PID_GAINS)
		} andThen openLoopGetToPositionCommand(CLIMBING_DOWN.setpoint, CLIMBING_DOWN.output) andThen
				maintainSetpointCommand(CLIMBING_DOWN.setpoint)
	}

/**
 * Climbing mechanism retracts open-loop until it passed [STAYING_FOLDED.setpoint],
 * then performs PID to maintain that setpoint. Command has no end condition.
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.stayFoldedCommand(): Command =
	withName("stay folded") {
		runOnce {
			configPIDF(Constants.NO_WEIGHT_BEARING_PID_GAINS)
		} andThen openLoopGetToPositionCommand(STAYING_FOLDED.setpoint, STAYING_FOLDED.output) andThen
				maintainSetpointCommand(STAYING_FOLDED.setpoint)
	}

/**
 * [percentOutput] is assumed -1 to 1, will come from joysticks.
 *
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.openLoopTeleopCommand(percentOutput: () -> Double): Command =
	withName("climbing open loop teleop") {
		run {
			setSpeed(percentOutput())
		} finallyDo {
			setSpeed(0.0)
		}
	}

/** [changeInPosition] is assumed -1 to 1, will come from joysticks.
 * Modify the rate of change using [multiplier].
 *
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.closedLoopTeleopCommand(changeInPosition: () -> Double, multiplier: Double): Command =
	withName("climbing closed loop teleop") {
		run {
			val delta = changeInPosition() * multiplier
			increasePositionSetpointBy(delta)
		} finallyDo {
			setSpeed(0.0)
		}
	}

/**
 * To be used in testing or in other command groups.
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.maintainSetpointCommand(setpoint: Rotations): Command =
	withName("maintain setpoint") {
		run { setPositionSetpoint(setpoint) }
	}

/**
 * To be used in testing or in other command groups.
 * - Requirements: climbing.
 */
private fun ClimbingSubsystem.openLoopGetToPositionCommand(desiredPosition: Rotations, output: Double): Command {
	val endCondition: (() -> Boolean)?

	if (output < 0.0) {
		endCondition = { currentPosition < desiredPosition }
	} else {
		endCondition = { currentPosition > desiredPosition }
	}

	return withName("get to setpoint") {
		run {
			setSpeed(output)
		} until endCondition
	}
}