package frc.robot.commands

import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_ANGLE_FOR_INTAKE
import frc.robot.subsystems.shooter.ShooterConstants.SHOOT_TIME_SEC
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem

/**
 * SHOULD BE THE DEFAULT COMMAND OF SHOOTER SUBSYSTEM.
 */
fun prepareShooterForIntakingCommand(): Command {
    return getToShooterStateCommand(ShooterState(SHOOTER_ANGLE_FOR_INTAKE, AngularVelocity.fromRpm(0.0)))
}

fun intakeCommand() {
    prepareShooterForIntakingCommand().alongWith(intakeIntoLoaderCommand())
}

/**
 * To be used in a command group.
 */
fun waitUntilShooterInToleranceCommand() = WaitUntilCommand(ShooterSubsystem::withinTolerance)

/**
 * No end condition. This is intentional.
 */
fun getToShooterStateCommand(state: ShooterState): Command {
    return ShooterSubsystem.run { ShooterSubsystem.setShooterState(state) }
}

fun loadAndShootCommand(state: ShooterState): Command {
    return getToShooterStateCommand(state).raceWith(loadIntoShooterCommand())
}


// ------ Other than for testing, the commands below should only be used in other command groups in this file. ------

/**
 * Apart from testing,
 * Should only be used in [intakeCommand].
 */
fun IntakeSubsystem.runIntakeCommand(): Command {
    return run { set(IntakeConstants.DEFAULT_OUTPUT) }
        .finallyDo { set(0.0) }
}

/**
 * Apart from testing,
 * Should only be used in [intakeIntoLoaderCommand] or [loadIntoShooterCommand].
 */
fun runLoaderCommand(): Command {
    return run { LoaderSubsystem.setLoader(ShooterConstants.LOADER_OUTPUT) }
        .finallyDo { LoaderSubsystem.setLoader(0.0) }
}

/**
 * Apart from testing,
 * Should only be used in [intakeCommand].
 */
fun intakeIntoLoaderCommand(): Command {
    return IntakeSubsystem.runIntakeCommand() andThen runLoaderCommand()
        .withTimeout(ShooterConstants.LOAD_FROM_INTAKE_TIME_SEC)
}

/**
 * Apart from testing,
 * Should only be used in [loadAndShootCommand].
 */
fun loadIntoShooterCommand(): Command {
    return waitUntilShooterInToleranceCommand()
        .andThen(runLoaderCommand())
        .withTimeout(SHOOT_TIME_SEC)
}

// -------------------------------------------- Manual overrides ---------------------------------------------------

fun ShooterSubsystem.openLoopTeleop_shooterAngle(percentOutput: () -> Double): Command {
    return run { setAngleMotorOutput(percentOutput()) }.finallyDo { setAngleMotorOutput(0.0) }
}

/**
 * [changeInAngleSupplier] is assumed -1 to 1, will come from joysticks.
 */
fun ShooterSubsystem.closedLoopTeleop_shooterAngle(changeInAngleSupplier: () -> Double, multiplier: Double): Command {
    return run {
        increaseAngleSetpointBy(Rotation2d.fromDegrees(changeInAngleSupplier() * multiplier))
    }
}

fun ShooterSubsystem.openLoopTeleop_shooterVelocity(percentOutput: () -> Double): Command {
    return run { increaseShooterMotorsOutputBy(percentOutput()) }.finallyDo { setShooterMotorsOutput(0.0) }
}

/**
 * [changeInVelocitySupplier] is assumed -1 to 1, will come from joysticks.
 */
fun ShooterSubsystem.closedLoopTeleop_shooterVelocity(
    changeInVelocitySupplier: () -> Double,
    multiplier: Double
): Command {
    return run { increaseVelocitySetpointBy(AngularVelocity.fromRpm(changeInVelocitySupplier() * multiplier)) }
}