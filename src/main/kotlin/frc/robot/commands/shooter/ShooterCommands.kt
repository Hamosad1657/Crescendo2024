package frc.robot.commands.shooter

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
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

fun ShooterSubsystem.shootCommand(): Command {}