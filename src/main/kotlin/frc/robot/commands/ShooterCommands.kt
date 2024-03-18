package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.PercentOutput
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.shooter.DynamicShooting
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterConstants.ESCAPE_ANGLE_LOCK_DURATION
import frc.robot.subsystems.shooter.ShooterConstants.ESCAPE_ANGLE_LOCK_OUTPUT
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.ShooterSubsystem as Shooter
import frc.robot.subsystems.swerve.SwerveSubsystem as Swerve

/**
 * - Command has no end condition.
 * - Requirements: Shooter.
 */
fun Shooter.getToShooterStateCommand(state: ShooterState): Command = withName("get to shooter state") {
	runOnce {
		resetVelocityPIDController()
	} andThen run {
		setShooterState(state)
	} finallyDo {
		stopShooterMotors()
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Shooter.
 */
fun Shooter.getToShooterStateCommand(state: () -> ShooterState): Command = withName("get to shooter state") {
	runOnce {
		resetVelocityPIDController()
	} andThen run {
		setShooterState(state())
	} finallyDo {
		stopShooterMotors()
	}
}

fun Shooter.getToAtSpeakerState(): Command = withName("get to at speaker state") {
	getToShooterStateCommand {
		SmartDashboard.putBoolean("Is facing SPEAKER", Swerve.isFacingSpeaker)
		if (Swerve.isFacingSpeaker) ShooterState.AT_SPEAKER
		else ShooterState.REVERSE_AT_SPEAKER
	}
}

/** - Requirements: Shooter. */
fun Shooter.dynamicShootingCommand() = withName("dynamic shooting") {
	getToShooterStateCommand {
		val estimatedTranslation = Swerve.robotPose.translation
		DynamicShooting.calculateShooterState(estimatedTranslation)
	}
}

/**
 * - Command has no end condition.
 * - Requirements: Shooter.
 */
fun Shooter.getToAngleCommand(angle: Rotation2d): Command = withName("get to shooter state") {
	run {
		setAngle(angle)
	} finallyDo {
		stopShooterMotors()
	}
}

/**
 * Maintains [ShooterConstants.ShooterState.AUTO_COLLECT].
 * - Requirements: Shooter.
 */
fun Shooter.autoDefaultCommand(): Command = withName("auto get to state collect") {
	getToShooterStateCommand(ShooterState.AUTO_COLLECT)
}

/**
 * Maintains [ShooterConstants.ShooterState.COLLECT].
 * - Requirements: Shooter.
 */
fun Shooter.teleopDefaultCommand(): Command = withName("teleop get to state collect") {
	getToShooterStateCommand(ShooterState.COLLECT)
}

/**
 * Run the shooter angle motor at high speed for a short duration to get it out of the angle lock.
 * - Requirements: Shooter.
 */
fun Shooter.escapeAngleLockCommand(): Command = withName("escape angle lock") {
	run {
		setAngleMotorOutput(ESCAPE_ANGLE_LOCK_OUTPUT)
	} withTimeout ESCAPE_ANGLE_LOCK_DURATION finallyDo {
		stopAngleMotor()
	}
}


// ---
//
// Manual overrides
//
// ---

/** - Requirements: Shooter. */
fun Shooter.openLoopTeleop_shooterAngle(
	output: () -> PercentOutput,
): Command = withName("angle open loop teleop") {
	run {
		setAngleMotorOutput(output())
	} finallyDo {
		stopAngleMotor()
	}
}

/**
 * [changeInAngle] is assumed -1 to 1, will come from joysticks.
 * To modify the rate of change, use [multiplier].
 *
 * - Requirements: Shooter.
 */
fun Shooter.closedLoopTeleop_shooterAngle(
	changeInAngle: () -> Double, multiplier: Double,
): Command = withName("angle closed loop teleop") {
	run {
		val delta = changeInAngle() * multiplier
		increaseAngleSetpointBy(Rotation2d.fromDegrees(delta))
	}
}

/** - Requirements: Shooter. */
fun Shooter.openLoopTeleop_shooterVelocity(
	output: () -> PercentOutput,
): Command = withName("velocity open loop teleop") {
	run {
		setShooterMotorsOutput(output())
	} finallyDo {
		stopShooterMotors()
	}
}

/**
 * [changeInVelocity] is assumed -1 to 1, will come from joysticks.
 * To modify the rate of change, use [multiplier].
 *
 * - Requirements: Shooter.
 */
fun Shooter.closedLoopTeleop_shooterVelocity(
	changeInVelocity: () -> Double, multiplier: Double,
): Command = withName("velocity closed loop teleop") {
	run {
		val delta = changeInVelocity() * multiplier
		increaseVelocitySetpointBy(AngularVelocity.fromRpm(delta))
	}
}
