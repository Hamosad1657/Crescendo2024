package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Rotations
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants
import frc.robot.subsystems.climbing.ClimbingSubsystem

fun ClimbingSubsystem.getToOpenLimitCommand(): Command = withName("open loop get to open limit") {
	run { set(ClimbingConstants.REACH_UP_OUTPUT) } until { isAtOpenedLimit } finallyDo { stop() }
}

fun ClimbingSubsystem.getToClosedLimitCommand(): Command = withName("open loop get to closed limit") {
	run { set(ClimbingConstants.REACH_DOWN_OUTPUT) } until { isAtClosedLimit } finallyDo { stop() }
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