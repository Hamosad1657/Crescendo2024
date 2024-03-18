package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.PercentOutput
import edu.wpi.first.wpilibj2.command.Command
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
			setLeft(leftOutput())
			setRight(rightOutput())
		} finallyDo {
			stopMotors()
		}
	}
