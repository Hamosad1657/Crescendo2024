package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterSubsystem

/**
 * To be used in a command group.
 */
fun ShooterSubsystem.loadCommand(): Command {
    
}

/**
 * To be used in a command group.
 */
fun ShooterSubsystem.waitUntilShooterInTolerance() = WaitUntilCommand {}

/**
 * No end condition. To be used in a command group.
 */
fun ShooterSubsystem.getToShooterStateCommand(): Command {}

/**
 * Time between when loading started to when the note is shot.
 * It might be a little different in different speeds, so put here
 * it's maximum value.
 */
private const val SHOOT_TIME = 0.0 // TODO: Measure SHOOT_TIME

fun ShooterSubsystem.shootCommand(): Command {}