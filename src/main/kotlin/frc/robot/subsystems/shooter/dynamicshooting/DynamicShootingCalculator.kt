package frc.robot.subsystems.shooter.dynamicshooting

import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SHOOTER_ANGLE_OFFSET
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SHOOTER_HEIGHT
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SHOOTER_PIVOT_TO_CHASSIS_CENTER
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_BLUE_POSITION_METERS
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_HEIGHT
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_RED_POSITION_METERS
import kotlin.math.atan2

// Everything that says "shooter position" is referring to the center of the shooter along its pivot axis.
// All of the position and distance units are meters.

/** This function assumes the robot is directly facing the speaker. */
fun calculateShooterState(chassisPosition: Translation2d, alliance: Alliance): ShooterState {
	val speakerPosition = determineSpeakerPosition(alliance)
	val shooterPosition = calculateShooterPosition(chassisPosition)

	val shooterToSpeakerFlatDistance = shooterPosition.getDistance(speakerPosition)
	val requiredShooterAngle = calculateRequiredShooterAngle(shooterToSpeakerFlatDistance)

	val angleSetpoint = requiredShooterAngle + SHOOTER_ANGLE_OFFSET
	val velocitySetpoint = calculateVelocitySetpoint(requiredShooterAngle, shooterToSpeakerFlatDistance)

	return ShooterState(angleSetpoint, velocitySetpoint)
}

private fun determineSpeakerPosition(alliance: Alliance) =
	when (alliance) {
		Alliance.Red -> SPEAKER_RED_POSITION_METERS
		Alliance.Blue -> SPEAKER_BLUE_POSITION_METERS
	}

private fun calculateShooterPosition(chassisPosition: Translation2d) =
	chassisPosition + SHOOTER_PIVOT_TO_CHASSIS_CENTER

private fun calculateRequiredShooterAngle(shooterToSpeakerFlatDistance: Double): Rotation2d {
	val shooterToSpeakerHeightDifference = SPEAKER_HEIGHT.meters - SHOOTER_HEIGHT.meters

	return Rotation2d.fromRadians(
		atan2(shooterToSpeakerFlatDistance, shooterToSpeakerHeightDifference)
	)
}

private fun calculateVelocitySetpoint(
	shooterToSpeakerAngle: Rotation2d,
	shooterToSpeakerFlatDistance: Double,
): AngularVelocity {
	// TODO: Implement (by using a function or shooting ranges)
	return AngularVelocity.fromRpm(0.0)
}
