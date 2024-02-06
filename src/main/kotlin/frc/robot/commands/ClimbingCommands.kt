package frc.robot.commands

import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.until
import com.hamosad1657.lib.commands.withName
import com.hamosad1657.lib.units.FractionalOutput
import com.hamosad1657.lib.units.Rotations
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants.ClimbingState.*
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

/**
 * A small constant output is applied to keep the climbing mechanism in place.
 * Command has no end condition.
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.openLoopStayFoldedCommand(): Command =
	withName("open loop stay folded") {
		run {
			set(Constants.STAY_FOLDED_OUTPUT)
		}
	}

/**
 * PID is performed to keep climbing mechanism at [FOLDING.setpoint].
 * Command has no end condition.
 */
fun ClimbingSubsystem.closedLoopStayFoldedCommand(): Command =
	withName("closed loop stay folded") {
		runOnce {
			configPIDF(Constants.NO_WEIGHT_BEARING_PID_GAINS)
		} andThen maintainSetpointCommand(FOLDING.setpoint)
	}

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
 * Climbing mechanism retracts open-loop until it passed [FOLDING.setpoint].
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.foldCommand(): Command =
	withName("fold") {
		runOnce {
			configPIDF(Constants.NO_WEIGHT_BEARING_PID_GAINS)
		} andThen openLoopGetToPositionCommand(FOLDING.setpoint, FOLDING.output)
	}

/**
 * Climbing mechanism retracts open-loop until it passed [FOLDING.setpoint],
 * then performs PID to maintain that setpoint. Command has no end condition.
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.foldAndStayFolded(): Command =
	withName("fold and stay folded") {
		foldCommand() andThen closedLoopStayFoldedCommand()
	}

/**
 * [percentOutput] is assumed -1 to 1, will come from joysticks.
 *
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.openLoopTeleopCommand(output: () -> FractionalOutput): Command =
	withName("climbing open loop teleop") {
		run {
			set(output())
		} finallyDo {
			set(0.0)
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
			set(0.0)
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
private fun ClimbingSubsystem.openLoopGetToPositionCommand(
	desiredPosition: Rotations,
	output: FractionalOutput
): Command {
	val endCondition: (() -> Boolean)

	if (output < 0.0) {
		endCondition = { currentPosition <= desiredPosition }
	} else {
		endCondition = { currentPosition >= desiredPosition }
	}

	return withName("get to setpoint") {
		run {
			set(output)
		} until endCondition
	}
}