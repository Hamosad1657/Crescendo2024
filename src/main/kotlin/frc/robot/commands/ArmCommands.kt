package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.PercentOutput
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.arm.ArmConstants.FOLD_OUTPUT
import frc.robot.subsystems.arm.ArmConstants.OPEN_OUTPUT
import frc.robot.subsystems.arm.ArmConstants.STAY_FOLDED_OUTPUT
import frc.robot.subsystems.arm.ArmSubsystem

/** - Requirements: Arm. */
fun ArmSubsystem.openCommand(): Command = withName("open") {
	run {
		setBothMotors(OPEN_OUTPUT)
	} until ::isAtLimit finallyDo {
		setBothMotors(0.0)
	}
}

/** - Requirements: Arm. */
fun ArmSubsystem.foldCommand(): Command = withName("fold") {
	run {
		setBothMotors(FOLD_OUTPUT)
	} until ::isAtLimit finallyDo {
		setBothMotors(0.0)
	}
}

/** - Requirements: Arm. */
fun ArmSubsystem.stayFoldedCommand(): Command = withName("stay folded") {
	run {
		setBothMotors(STAY_FOLDED_OUTPUT)
	} finallyDo {
		setBothMotors(0.0)
	}
}

/** - Requirements: Arm. */
fun ArmSubsystem.foldAndStayFoldedCommand(): Command =
	withName("fold & stay folded") {
		foldCommand() andThen
			stayFoldedCommand()
	}

/** - Requirements: Arm. */
fun ArmSubsystem.separateSidesTeleopCommand(
	leftOutput: () -> PercentOutput,
	rightOutput: () -> PercentOutput
): Command = withName("separate sides teleop") {
	run {
		setLeftMotor(leftOutput())
		setRightMotor(rightOutput())
	} finallyDo {
		setBothMotors(0.0)
	}
}

/** - Requirements: Arm. */
fun ArmSubsystem.bothSidesTeleopCommand(output: () -> PercentOutput) = withName("both sides teleop") {
	separateSidesTeleopCommand(output, output)
}