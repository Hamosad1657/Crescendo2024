package frc.robot.subsystems.shooter.dynamicshooting

import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Length
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SHOOTER_PIVOT_TO_CHASSIS_CENTER_TRANSLATION3D
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_POSITION_METERS_BLUE
import frc.robot.subsystems.shooter.dynamicshooting.DynamicShootingConstants.SPEAKER_POSITION_METERS_RED
import kotlin.math.atan2

/** This function assumes the robot is directly facing the speaker. */
fun calculateShooterState(chassisPositionMeters: Translation2d, alliance: Alliance): ShooterState {
	val speakerPosition3dMeters = determineSpeakerPosition(alliance)
	val shooterPosition3dMeters = calculateShooterPosition3dMeters(chassisPositionMeters)

	val shooterToSpeakerPitch =
		
}

private fun calculateVelocitySetpoint(
	shooterToSpeakerPitch: Rotation2d,
	shooterToSpeakerFlatDistance: Length
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

private fun calculateShooterPosition3dMeters(chassisPositionMeters: Translation2d): Translation3d {
	val robotPosition3dMeters = Translation3d(chassisPositionMeters.x, chassisPositionMeters.y, 0.0)
	return robotPosition3dMeters + SHOOTER_PIVOT_TO_CHASSIS_CENTER_TRANSLATION3D
}

private fun calculateShooterToSpeakerTranslation3d(
	chassisPositionMeters: Translation2d,
	alliance: Alliance
): Translation3d {
	val shooterPosition3dMeters = calculateShooterPosition3dMeters(chassisPositionMeters)
	val speakerPosition3dMeters = determineSpeakerPosition(alliance)

	return speakerPosition3dMeters.minus(shooterPosition3dMeters)
}