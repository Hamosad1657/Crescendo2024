package frc.robot.commands

import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.until
import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.arm.ArmConstants.FOLD_OUTPUT
import frc.robot.subsystems.arm.ArmConstants.OPEN_OUTPUT
import frc.robot.subsystems.arm.ArmConstants.STAY_FOLDED_OUTPUT
import frc.robot.subsystems.arm.ArmSubsystem

/** - Requirements: arm. */
fun ArmSubsystem.openCommand(): Command {
	return withName("open") {
		run { setWithLimits(OPEN_OUTPUT) } until ::isAtForwardLimit finallyDo { set(0.0) }
	}
}

/** - Requirements: arm. */
fun ArmSubsystem.foldCommand(): Command {
	return withName("fold") {
		run { setWithLimits(FOLD_OUTPUT) } until ::isAtReverseLimit finallyDo { set(0.0) }
	}
}

fun ArmSubsystem.stayFoldedCommand(): Command {
	return withName("stay folded") {
		run { set(STAY_FOLDED_OUTPUT) } finallyDo { set(0.0) }
	}
}

fun ArmSubsystem.foldAndStayFoldedCommand(): Command {
	return withName("fold & stay folded") { foldCommand() andThen stayFoldedCommand() }
}