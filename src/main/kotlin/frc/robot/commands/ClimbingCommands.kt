package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Rotations
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants.ClimbingState.*
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

/**
 * A small constant output is applied to keep the climbing mechanism in place.
 * - Command has no end condition.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.openLoopStayFoldedCommand(): Command = withName("open loop stay folded") {
	run {
		set(Constants.STAY_FOLDED_OUTPUT)
	}
}

/**
 * PID is performed to keep climbing mechanism at [FOLDING].
 * - Command has no end condition.
 */
fun ClimbingSubsystem.closedLoopStayFoldedCommand(): Command = withName("closed loop stay folded") {
	runOnce {
		configPIDF(holdingRobot = FOLDING.holdingRobot)
	} andThen
		maintainSetpointCommand(FOLDING.setpoint)
}

/**
 * Climbing mechanism extends open-loop until it passed [REACHING_CHAIN], then the command ends.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.reachChainCommand(): Command = withName("reach chain") {
	runOnce {
		configPIDF(holdingRobot = REACHING_CHAIN.holdingRobot)
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
		configPIDF(holdingRobot = PULLING_UP_ROBOT.holdingRobot)
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
		configPIDF(holdingRobot = CLIMBING_DOWN.holdingRobot)
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
		configPIDF(holdingRobot = FOLDING.holdingRobot)
	} andThen
		openLoopGetToPositionCommand(FOLDING.setpoint, FOLDING.output)
}

/**
 * Climbing mechanism retracts open-loop until it passed [FOLDING],
 * then performs PID to maintain that setpoint.
 * - Command has no end condition.
 * - Requirements: Climbing.
 */
fun ClimbingSubsystem.foldAndStayFolded(): Command = withName("fold and stay folded") {
	foldCommand() andThen
		closedLoopStayFoldedCommand()
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
			set(0.0)
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
			set(0.0)
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