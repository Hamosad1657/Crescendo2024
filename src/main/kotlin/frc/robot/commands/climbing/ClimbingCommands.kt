package frc.robot.commands.climbing

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

fun ClimbingSubsystem.reachUpCommand(): Command {
    return runOnce { setMaxVelocity(Constants.ClimbingMode.REACHING_CHAIN.speedLimit) }.andThen(
        run { setMotionMagicVoltage(Constants.ClimbingMode.REACHING_CHAIN.setpointAndFF) })
        .until { withinTolerance() }
}

fun ClimbingSubsystem.climbUpCommand(): Command {
    return runOnce { setMaxVelocity(Constants.ClimbingMode.CLIMBING_UP.speedLimit) }.andThen(
        run { setMotionMagicVoltage(Constants.ClimbingMode.CLIMBING_UP.setpointAndFF) })
}

fun ClimbingSubsystem.climbDownCommand(): Command {
    return runOnce { setMaxVelocity(Constants.ClimbingMode.CLIMBING_DOWN.speedLimit) }.andThen(
        run { setMotionMagicVoltage(Constants.ClimbingMode.CLIMBING_DOWN.setpointAndFF) })
        .until { withinTolerance() }
}

fun ClimbingSubsystem.stayFoldedCommand(): Command {
    return runOnce { setMaxVelocity(Constants.ClimbingMode.STAYING_FOLDED.speedLimit) }.andThen(
        run { setMotionMagicVoltage(Constants.ClimbingMode.STAYING_FOLDED.setpointAndFF) })
}
