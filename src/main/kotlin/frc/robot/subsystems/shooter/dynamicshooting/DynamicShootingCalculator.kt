package frc.robot.subsystems.shooter.dynamicshooting

import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SHOOTER_PITCH_OFFSET
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SHOOTER_PIVOT_TO_CHASSIS_CENTER_TRANSLATION3D
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_HEIGHT
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_POSITION_METERS_BLUE
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_POSITION_METERS_RED
import kotlin.math.atan2

/** This function assumes the robot is directly facing the speaker. */
fun calculateShooterState(chassisPosition2dMeters: Translation2d, alliance: Alliance): ShooterState {
	val speakerPosition3dMeters = determineSpeakerPosition(alliance)
	val shooterPosition3dMeters = calculateShooterPosition3dMeters(chassisPosition2dMeters)

	val shooterToSpeakerFlatDistanceMeters =
		shooterPosition3dMeters.toTranslation2d()
			.getDistance(
				speakerPosition3dMeters.toTranslation2d()
			)

	val shooterToSpeakerHeightDifferenceMeters = SPEAKER_HEIGHT.meters - shooterPosition3dMeters.z

	val shooterToSpeakerPitch = Rotation2d.fromRadians(
		atan2(shooterToSpeakerFlatDistanceMeters, shooterToSpeakerHeightDifferenceMeters)
	)

	val angleSetpoint = shooterToSpeakerPitch + SHOOTER_PITCH_OFFSET
	val velocitySetpoint = calculateVelocitySetpoint(shooterToSpeakerPitch, shooterToSpeakerFlatDistanceMeters)
	return ShooterState(angleSetpoint, velocitySetpoint)
}

private fun calculateVelocitySetpoint(
	shooterToSpeakerPitch: Rotation2d,
	shooterToSpeakerFlatDistanceMeters: Double
): AngularVelocity {
	// TODO: Implement
	return AngularVelocity.fromRpm(0.0)
}

private fun determineSpeakerPosition(alliance: Alliance): Translation3d =
	if (alliance == Alliance.Blue) {
		SPEAKER_POSITION_METERS_BLUE
	} else {
		SPEAKER_POSITION_METERS_RED
	}

private fun calculateShooterPosition3dMeters(chassisPosition2dMeters: Translation2d): Translation3d {
	val chassisPosition3dMeters = Translation3d(chassisPosition2dMeters.x, chassisPosition2dMeters.y, 0.0)
	return chassisPosition3dMeters + SHOOTER_PIVOT_TO_CHASSIS_CENTER_TRANSLATION3D
}