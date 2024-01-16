package frc.robot.commands.arm

import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.until
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.arm.ArmSubsystem

fun ArmSubsystem.openTrapCommand(): Command {
    return run { set(DEFAULT_OUTPUT) } until ::isAtForwardLimit finallyDo { set(0.0) }
}

fun ArmSubsystem.foldCommand(): Command {
    return run { set(-DEFAULT_OUTPUT) } until ::isAtReverseLimit finallyDo { set(0.0) }
}
