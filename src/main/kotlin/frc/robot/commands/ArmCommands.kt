package frc.robot.commands

import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.until
import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.arm.ArmConstants.DEFAULT_OUTPUT
import frc.robot.subsystems.arm.ArmSubsystem

/** - Requirements: arm. */
fun ArmSubsystem.openCommand(): Command {
    return withName("open") {
        run { set(DEFAULT_OUTPUT) } until ::isAtForwardLimit finallyDo { set(0.0) }
    }
}

/** - Requirements: arm. */
fun ArmSubsystem.foldCommand(): Command {
    return withName("fold") {
        run { set(-DEFAULT_OUTPUT) } until ::isAtReverseLimit finallyDo { set(0.0) }
    }
}
