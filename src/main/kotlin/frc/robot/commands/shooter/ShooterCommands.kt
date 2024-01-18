package frc.robot.commands.shooter

import com.hamosad1657.lib.commands.andThen
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.subsystems.shooter.ShooterConstants.SHOOT_TIME
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.shooter.ShooterConstants as Constants

/**
 * To be used in a command group.
 */
fun ShooterSubsystem.runLoaderCommand(): Command = run { setLoader(Constants.LOADER_OUTPUT) }

/**
 * To be used in a command group.
 */
fun ShooterSubsystem.waitUntilShooterInTolerance() = WaitUntilCommand(::withinTolerance)

/**
 * To be used in a command group.
 */
fun ShooterSubsystem.loadWhenShooterInTolerance(): Command {
    return waitUntilShooterInTolerance() andThen runLoaderCommand().withTimeout(SHOOT_TIME)
}

/**
 * No end condition. This is intentional.
 */
fun ShooterSubsystem.getToShooterStateCommand(state: ShooterState): Command {
    return run { setShooterState(state) }
}

fun ShooterSubsystem.shootCommand(state: ShooterState): Command {
    return getToShooterStateCommand(state).raceWith(loadWhenShooterInTolerance())
}