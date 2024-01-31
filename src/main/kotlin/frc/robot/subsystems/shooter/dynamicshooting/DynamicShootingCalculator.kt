package frc.robot.subsystems.shooter.dynamicshooting

import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Length
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SHOOTER_END_FROM_ROBOT_CENTER_DISTANCE
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SHOOTER_END_HEIGHT
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SHOOTER_PITCH_OFFSET
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_POSITION_METERS_BLUE
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_POSITION_METERS_RED
import kotlin.math.atan2

/** This function assumes the robot is directly facing the speaker. */
fun calculateShooterState(robotPositionMeters: Translation2d, alliance: Alliance): ShooterState {
	val robotToSpeakerFlatDistance = calculateFlatDistanceToSpeaker(robotPositionMeters, alliance)
	val shooterEndToSpeakerFlatDistance = robotToSpeakerFlatDistance + SHOOTER_END_FROM_ROBOT_CENTER_DISTANCE

	val shooterToSpeakerPitch =
		Rotation2d.fromRadians(atan2(shooterEndToSpeakerFlatDistance.meters, SHOOTER_END_HEIGHT.meters))
	val angleSetpoint = shooterToSpeakerPitch + SHOOTER_PITCH_OFFSET

	val velocitySetpoint = calculateVelocitySetpoint(shooterToSpeakerPitch, shooterEndToSpeakerFlatDistance)

	return ShooterState(angleSetpoint, velocitySetpoint)
}

private fun calculateVelocitySetpoint(
	shooterToSpeakerPitch: Rotation2d,
	shooterEndToSpeakerFlatDistance: Length
): AngularVelocity {
	// TODO: Implement
	return AngularVelocity.fromRpm(0.0)
}

private fun calculateFlatDistanceToSpeaker(robotPositionMeters: Translation2d, alliance: Alliance): Length {
	val speakerPosition = if (alliance == Alliance.Blue) {
		SPEAKER_POSITION_METERS_BLUE
	} else {
		SPEAKER_POSITION_METERS_RED
	}
	return Length.fromMeters(robotPositionMeters.getDistance(speakerPosition))
}