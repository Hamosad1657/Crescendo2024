package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.PercentOutput
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants as Constants
import frc.robot.subsystems.climbing.ClimbingSubsystem as Climbing

/**
 * [output] is assumed -1 to 1, will come from joysticks.
 * - Requirements: Climbing.
 */
fun Climbing.openLoopTeleopCommand(output: () -> PercentOutput): Command =
	withName("climbing open loop teleop") {
		run {
			set(output())
		} finallyDo {
			stopMotors()
		}
	}

/**
 * [leftOutput] and [rightOutput] are assumed -1 to 1, will come from joysticks.
 * - Requirements: Climbing.
 */
fun Climbing.openLoopTeleopCommand(
	leftOutput: () -> PercentOutput,
	rightOutput: () -> PercentOutput,
): Command =
	withName("climbing open loop teleop") {
		run {
			setLeft(climbSpeed(leftOutput()))
			setRight(climbSpeed(rightOutput()))
		} finallyDo {
			stopMotors()
		}
	}

private fun climbSpeed(output: PercentOutput) =
	if (output > 0.0) output * Constants.OPEN_CLIMBING_OUTPUT
	else output * Constants.CLOSE_CLIMBING_OUTPUT
