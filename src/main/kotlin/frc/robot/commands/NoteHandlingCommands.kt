@file:Suppress("FunctionName")

package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.loader.LoaderConstants
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem

fun collectCommand(): Command =
    ShooterSubsystem.prepareShooterForCollectingCommand() alongWith
            collectIntoLoaderCommand()

/** SHOULD BE THE DEFAULT COMMAND OF SHOOTER SUBSYSTEM */
fun ShooterSubsystem.prepareShooterForCollectingCommand(): Command =
    getToShooterStateCommand(
        ShooterState(
            ShooterConstants.ANGLE_FOR_INTAKE,
            AngularVelocity.fromRpm(0.0)
        )
    )

fun loadAndShootCommand(state: ShooterState): Command =
    ShooterSubsystem.getToShooterStateCommand(state) raceWith
            loadIntoShooterCommand()

/** No end condition. This is intentional. */
fun ShooterSubsystem.getToShooterStateCommand(state: ShooterState): Command =
    run {
        setShooterState(state)
    }


// ---
//
// Other than for testing, the commands below should only be used in other command groups in this file.
//
// ---

/** Apart from testing, should only be used in [collectCommand] or in a manual override. */
fun IntakeSubsystem.runIntakeCommand(): Command =
    run {
        if (ShooterSubsystem.withinTolerance) {
            set(IntakeConstants.MOTOR_OUTPUT)
        } else {
            set(0.0)
        }
    } finallyDo { _ ->
        set(0.0)
    }


/** Apart from testing, should only be used in [collectIntoLoaderCommand] or [loadIntoShooterCommand], or in a manual override. */
fun LoaderSubsystem.runLoaderCommand(): Command =
    run {
        setLoader(LoaderConstants.MOTOR_OUTPUT)
    } finallyDo { _ ->
        setLoader(0.0)
    }


/** Apart from testing, should only be used in [collectCommand] or in a manual override. */
fun collectIntoLoaderCommand(): Command =
    IntakeSubsystem.runIntakeCommand() alongWith
            LoaderSubsystem.runLoaderCommand() until
            LoaderSubsystem::isNoteDetected


/** Apart from testing, should only be used in [loadAndShootCommand] or in a manual override. */
fun loadIntoShooterCommand(): Command =
    WaitUntilCommand(ShooterSubsystem::withinTolerance) andThen
            LoaderSubsystem.runLoaderCommand().withTimeout(ShooterConstants.SHOOT_TIME_SEC)


// ---
//
// Manual overrides
//
// ---

fun ShooterSubsystem.openLoopTeleop_shooterAngle(percentOutput: () -> Double): Command =
    run {
        setAngleMotorOutput(percentOutput())
    } finallyDo { _ ->
        setAngleMotorOutput(0.0)
    }


/** [changeInAngle] is assumed -1 to 1, will come from joysticks. */
fun ShooterSubsystem.closedLoopTeleop_shooterAngle(changeInAngle: () -> Double, multiplier: Double): Command =
    run {
        val delta = changeInAngle() * multiplier
        increaseAngleSetpointBy(Rotation2d.fromDegrees(delta))
    }


fun ShooterSubsystem.openLoopTeleop_shooterVelocity(percentOutput: () -> Double): Command =
    run {
        increaseShooterMotorsOutputBy(percentOutput())
    }.finallyDo { _ ->
        setShooterMotorsOutput(0.0)
    }


/** [changeInVelocity] is assumed -1 to 1, will come from joysticks. */
fun ShooterSubsystem.closedLoopTeleop_shooterVelocity(changeInVelocity: () -> Double, multiplier: Double): Command =
    run {
        val delta = changeInVelocity() * multiplier
        increaseVelocitySetpointBy(AngularVelocity.fromRpm(delta))
    }
