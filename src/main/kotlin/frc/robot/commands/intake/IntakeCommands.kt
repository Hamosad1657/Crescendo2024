package frc.robot.commands.intake

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.intake.IntakeConstants as Constants

fun IntakeSubsystem.intakeCommand(): Command {
    return run { set(Constants.OUTPUT) }
}
