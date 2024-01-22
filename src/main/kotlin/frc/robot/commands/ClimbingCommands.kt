package frc.robot.commands

import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.until
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants.ClimbingState
import frc.robot.subsystems.climbing.ClimbingSubsystem

fun ClimbingSubsystem.reachChainCommand(): Command =
    runOnce {
        setMaxVelocity(ClimbingState.REACHING_CHAIN.maxVelocity)
    } andThen run {
        setClimbingStateSetpoint(ClimbingState.REACHING_CHAIN)
    } until ::isWithinTolerance

fun ClimbingSubsystem.pullUpRobotCommand(): Command =
    runOnce {
        setMaxVelocity(ClimbingState.PULLING_UP_ROBOT.maxVelocity)
    } andThen run {
        setClimbingStateSetpoint(ClimbingState.PULLING_UP_ROBOT)
    }

fun ClimbingSubsystem.climbDownCommand(): Command =
    runOnce {
        setMaxVelocity(ClimbingState.CLIMBING_DOWN.maxVelocity)
    } andThen run {
        setClimbingStateSetpoint(ClimbingState.CLIMBING_DOWN)
    } until ::isWithinTolerance


fun ClimbingSubsystem.stayFoldedCommand(): Command =
    runOnce {
        setMaxVelocity(ClimbingState.STAYING_FOLDED.maxVelocity)
    } andThen run {
        setClimbingStateSetpoint(ClimbingState.STAYING_FOLDED)
    }

/** [percentOutput] is assumed -1 to 1, will come from joysticks. */
fun ClimbingSubsystem.openLoopTeleopCommand(percentOutput: () -> Double): Command =
    run {
        setSpeed(percentOutput())
    } finallyDo {
        setSpeed(0.0)
    }

/** [changeInPosition] is assumed -1 to 1, will come from joysticks.
 * Modify the rate of change using [multiplier].
 */
fun ClimbingSubsystem.closedLoopTeleopCommand(changeInPosition: () -> Double, multiplier: Double): Command =
    run {
        val delta = changeInPosition() * multiplier
        increasePositionSetpointBy(delta)
    } finallyDo {
        setSpeed(0.0)
    }
