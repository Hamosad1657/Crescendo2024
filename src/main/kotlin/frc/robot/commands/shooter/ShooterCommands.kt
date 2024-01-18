package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.subsystems.shooter.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterConstants.SHOOT_TIME
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.shooter.ShooterConstants as Constants

/**
 * To be used in a command group.
 */
fun runLoaderCommand(): Command {
    return run { LoaderSubsystem.setLoader(Constants.LOADER_OUTPUT) }
        .finallyDo { LoaderSubsystem.setLoader(0.0) }
}

/**
 * To be used in a command group.
 */
fun waitUntilShooterInTolerance() = WaitUntilCommand(ShooterSubsystem::withinTolerance)

/**
 * To be used in a command group.
 */
fun loadWhenShooterInTolerance(): Command {
    return waitUntilShooterInTolerance()
        .andThen(runLoaderCommand())
        .withTimeout(SHOOT_TIME)
}

/**
 * No end condition. This is intentional.
 */
fun getToShooterStateCommand(state: ShooterState): Command {
    return ShooterSubsystem.run { ShooterSubsystem.setShooterState(state) }
}

fun loadAndShootCommand(state: ShooterState): Command {
    return getToShooterStateCommand(state).raceWith(loadWhenShooterInTolerance())
}