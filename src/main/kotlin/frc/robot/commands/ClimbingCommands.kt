package frc.robot.commands

import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.commands.finallyDo
import com.hamosad1657.lib.commands.until
import com.hamosad1657.lib.commands.withName
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.climbing.ClimbingConstants.ClimbingState
import frc.robot.subsystems.climbing.ClimbingSubsystem

/** - Requirements: climbing. */
fun ClimbingSubsystem.reachChainCommand(): Command =
    withName("reach chain") {
        runOnce {
            setMaxVelocity(ClimbingState.REACHING_CHAIN.maxVelocity)
        } andThen run {
            setClimbingStateSetpoint(ClimbingState.REACHING_CHAIN)
        } until ::isWithinTolerance
    }

/** - Requirements: climbing. */
fun ClimbingSubsystem.pullUpRobotCommand(): Command =
    withName("pull up robot") {
        runOnce {
            setMaxVelocity(ClimbingState.PULLING_UP_ROBOT.maxVelocity)
        } andThen run {
            setClimbingStateSetpoint(ClimbingState.PULLING_UP_ROBOT)
        }
    }

/** - Requirements: climbing. */
fun ClimbingSubsystem.climbDownCommand(): Command =
    withName("climb down") {
        runOnce {
            setMaxVelocity(ClimbingState.CLIMBING_DOWN.maxVelocity)
        } andThen run {
            setClimbingStateSetpoint(ClimbingState.CLIMBING_DOWN)
        } until ::isWithinTolerance
    }

/** - Requirements: climbing. */
fun ClimbingSubsystem.stayFoldedCommand(): Command =
    withName("stay folded") {
        runOnce {
            setMaxVelocity(ClimbingState.STAYING_FOLDED.maxVelocity)
        } andThen run {
            setClimbingStateSetpoint(ClimbingState.STAYING_FOLDED)
        }
    }

/**
 * [percentOutput] is assumed -1 to 1, will come from joysticks.
 *
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.openLoopTeleopCommand(percentOutput: () -> Double): Command =
    withName("climbing open loop teleop") {
        run {
            setSpeed(percentOutput())
        } finallyDo {
            setSpeed(0.0)
        }
    }

/** [changeInPosition] is assumed -1 to 1, will come from joysticks.
 * Modify the rate of change using [multiplier].
 *
 * - Requirements: climbing.
 */
fun ClimbingSubsystem.closedLoopTeleopCommand(changeInPosition: () -> Double, multiplier: Double): Command =
    withName("climbing closed loop teleop") {
        run {
            val delta = changeInPosition() * multiplier
            increasePositionSetpointBy(delta)
        } finallyDo {
            setSpeed(0.0)
        }
    }
