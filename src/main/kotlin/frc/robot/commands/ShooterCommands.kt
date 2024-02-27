package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.PercentOutput
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.shooter.ShooterConstants
import frc.robot.subsystems.shooter.ShooterConstants.ESCAPE_ANGLE_LOCK_OUTPUT
import frc.robot.subsystems.shooter.ShooterConstants.TIME_TO_ESCAPE_ANGLE_LOCK_SEC
import frc.robot.subsystems.shooter.ShooterSubsystem


/**
 * Maintains [ShooterConstants.ShooterState.AUTO_COLLECT].
 * - Requirements: Shooter.
 */
fun ShooterSubsystem.autoDefaultCommand(): Command = withName("auto get to state collect") {
	getToShooterStateCommand(ShooterConstants.ShooterState.AUTO_COLLECT)
}

/**
 * Maintains [ShooterConstants.ShooterState.COLLECT].
 * - Requirements: Shooter.
 */
fun ShooterSubsystem.teleopDefaultCommand(): Command = withName("teleop get to state collect") {
	getToShooterStateCommand(ShooterConstants.ShooterState.COLLECT)
}

/**
 * Run the shooter angle motor at high speed for a short duration to get it out of the angle lock.
 * - Requirements: Shooter.
 */
fun ShooterSubsystem.escapeAngleLock(): Command = withName("escape angle lock") {
	run {
		setAngleMotorOutput(ESCAPE_ANGLE_LOCK_OUTPUT)
	} withTimeout TIME_TO_ESCAPE_ANGLE_LOCK_SEC finallyDo {
		stopAngleMotor()
	}
}


// ---
//
// Manual overrides
//
// ---

/** - Requirements: Shooter. */
fun ShooterSubsystem.openLoopTeleop_shooterAngle(
	output: () -> PercentOutput
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
fun ShooterSubsystem.closedLoopTeleop_shooterAngle(
	changeInAngle: () -> Double, multiplier: Double
): Command = withName("angle closed loop teleop") {
	run {
		val delta = changeInAngle() * multiplier
		increaseAngleSetpointBy(Rotation2d.fromDegrees(delta))
	}
}

/** - Requirements: Shooter. */
fun ShooterSubsystem.openLoopTeleop_shooterVelocity(
	output: () -> PercentOutput
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
fun ShooterSubsystem.closedLoopTeleop_shooterVelocity(
	changeInVelocity: () -> Double, multiplier: Double
): Command = withName("velocity closed loop teleop") {
	run {
		val delta = changeInVelocity() * multiplier
		increaseVelocitySetpointBy(AngularVelocity.fromRpm(delta))
	}
}



