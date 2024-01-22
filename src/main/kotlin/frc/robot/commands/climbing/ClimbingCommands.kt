package frc.robot.commands.climbing

import com.hamosad1657.lib.commands.finallyDo
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingSubsystem
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

fun ClimbingSubsystem.reachUpCommand(): Command {
    return runOnce { setMaxVelocity(Constants.ClimbingState.REACHING_CHAIN.maxVelocity) }.andThen(
        run { setClimbingStateSetpoint(Constants.ClimbingState.REACHING_CHAIN) })
        .until { withinTolerance }
}

fun ClimbingSubsystem.climbUpCommand(): Command {
    return runOnce { setMaxVelocity(Constants.ClimbingState.CLIMBING_UP.maxVelocity) }.andThen(
        run { setClimbingStateSetpoint(Constants.ClimbingState.CLIMBING_UP) })
}

fun ClimbingSubsystem.climbDownCommand(): Command {
    return runOnce { setMaxVelocity(Constants.ClimbingState.CLIMBING_DOWN.maxVelocity) }.andThen(
        run { setClimbingStateSetpoint(Constants.ClimbingState.CLIMBING_DOWN) })
        .until { withinTolerance }
}

fun ClimbingSubsystem.stayFoldedCommand(): Command {
    return runOnce { setMaxVelocity(Constants.ClimbingState.STAYING_FOLDED.maxVelocity) }.andThen(
        run { setClimbingStateSetpoint(Constants.ClimbingState.STAYING_FOLDED) })
}

/** [percentOutputSupplier] is assumed -1 to 1, will come from joysticks. */
fun ClimbingSubsystem.openLoopTeleopCommand(percentOutputSupplier: () -> Double): Command {
    return run { setSpeed(percentOutputSupplier()) } finallyDo { setSpeed(0.0) }
}

/** [changeInPositionSupplier] is assumed -1 to 1, will come from joysticks.
 * Modify the rate of change using [multiplier].
 */
fun ClimbingSubsystem.closedLoopTeleopCommand(changeInPositionSupplier: () -> Double, multiplier: Double): Command {
    return run {
        val delta = changeInPositionSupplier() * multiplier
        increasePositionSetpointBy(delta)
    }
}
