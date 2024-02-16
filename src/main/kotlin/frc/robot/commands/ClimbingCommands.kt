package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Rotations
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants
import frc.robot.subsystems.climbing.ClimbingConstants.ClimbingState.*
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

fun ClimbingSubsystem.openLoopGetToOpenLimitCommand(): Command = withName("open loop get to open limit") {
	run { set(ClimbingConstants.REACH_UP_OUTPUT) } until { isAtOpenedLimit } finallyDo { stop() }
}

fun ClimbingSubsystem.openLoopGetToClosedLimitCommand(): Command = withName("open loop get to closed limit") {
	run { set(ClimbingConstants.REACH_DOWN_OUTPUT) } until { isAtClosedLimit } finallyDo { stop() }
}

/**
 * A small constant output is applied to keep the climbing mechanism in place.
 * - Command has no end condition.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.openLoop_StayFoldedCommand(): Command = withName("open loop stay folded") {
	run {
		set(Constants.STAY_FOLDED_OUTPUT)
	}
}

/**
 * PID is performed to keep climbing mechanism at [FOLDING].
 * - Command has no end condition.
 */
fun ClimbingSubsystem.closedLoop_StayFoldedCommand(): Command = withName("closed loop stay folded") {
	runOnce {
		configPID(isHoldingRobot = FOLDING.isHoldingRobot)
	} andThen
		maintainSetpointCommand(FOLDING.setpoint)
}

/**
 * Climbing mechanism extends open-loop until it passed [REACHING_CHAIN], then the command ends.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.reachChainCommand(): Command = withName("reach chain") {
	runOnce {
		configPID(isHoldingRobot = REACHING_CHAIN.isHoldingRobot)
	} andThen
		openLoopGetToPositionCommand(REACHING_CHAIN.setpoint, REACHING_CHAIN.output)
}

/**
 * Climbing mechanism retracts open-loop until it passed [PULLING_UP_ROBOT],
 * then performs PID control to maintain that setpoint.
 * - Command has no end condition.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.pullUpRobotCommand(): Command = withName("pull up robot") {
	runOnce {
		configPID(isHoldingRobot = PULLING_UP_ROBOT.isHoldingRobot)
	} andThen
		openLoopGetToPositionCommand(PULLING_UP_ROBOT.setpoint, PULLING_UP_ROBOT.output) andThen
		maintainSetpointCommand(PULLING_UP_ROBOT.setpoint)
}

/**
 * Climbing mechanism extends open-loop until it passed [CLIMBING_DOWN],
 * then performs PID to maintain that setpoint.
 * - Command has no end condition.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.climbDownCommand(): Command = withName("climb down") {
	runOnce {
		configPID(isHoldingRobot = CLIMBING_DOWN.isHoldingRobot)
	} andThen
		openLoopGetToPositionCommand(CLIMBING_DOWN.setpoint, CLIMBING_DOWN.output) andThen
		maintainSetpointCommand(CLIMBING_DOWN.setpoint)
}

/**
 * Climbing mechanism retracts open-loop until it passed [FOLDING].
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.foldCommand(): Command = withName("fold") {
	runOnce {
		configPID(isHoldingRobot = FOLDING.isHoldingRobot)
	} andThen
		openLoopGetToPositionCommand(FOLDING.setpoint, FOLDING.output)
}

/**
 * Climbing mechanism retracts open-loop until it passed [FOLDING],
 * then performs PID to maintain that setpoint.
 * - Command has no end condition.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.openLoop_FoldAndStayFolded(): Command = withName("fold and stay folded") {
	foldCommand() andThen
		openLoop_StayFoldedCommand()
}

/**
 * [output] is assumed -1 to 1, will come from joysticks.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.openLoopTeleopCommand(output: () -> PercentOutput): Command =
	withName("climbing open loop teleop") {
		run {
			set(output())
		} finallyDo {
			stop()
		}
	}

/**
 * [output] is assumed -1 to 1, will come from joysticks.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.openLoopTeleopCommand(
	leftOutput: () -> PercentOutput,
	rightOutput: () -> PercentOutput
): Command =
	withName("climbing open loop teleop") {
		run {
			setLeft(leftOutput())
			setRight(rightOutput())
		} finallyDo {
			stop()
		}
	}

/**
 * [changeInPosition] is assumed -1 to 1, will come from joysticks.
 * Modify the rate of change using [multiplier].
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.closedLoopTeleopCommand(changeInPosition: () -> PercentOutput, multiplier: Double): Command =
	withName("climbing closed loop teleop") {
		run {
			val delta = changeInPosition() * multiplier
			increasePositionSetpointBy(delta)
		} finallyDo {
			stop()
		}
	}

/**
 * To be used in testing or in other command groups.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.maintainSetpointCommand(setpoint: Rotations): Command = withName("maintain setpoint") {
	run {
		setPositionSetpoint(setpoint)
	}
}

/**
 * To be used in testing or in other command groups.
 * - Requirements: Climbing.
 */
private fun ClimbingSubsystem.openLoopGetToPositionCommand(
	desiredPosition: Rotations,
	output: PercentOutput
): Command {
	val endCondition =
		if (output < 0.0) {
			{ currentPosition <= desiredPosition }
		} else {
			{ currentPosition >= desiredPosition }
		}

	return withName("get to setpoint") {
		run {
			set(output)
		} until endCondition
	}
}