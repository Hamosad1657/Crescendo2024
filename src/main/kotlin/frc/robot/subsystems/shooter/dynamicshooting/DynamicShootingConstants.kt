package frc.robot.subsystems.shooter.dynamicshooting

import com.hamosad1657.lib.units.centimeters
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.feet
import edu.wpi.first.math.geometry.Translation3d

object DynamicShootingConstants {

	val SPEAKER_HEIGHT = 0.0.feet

	/**
	 * The Translation2d of the blue speaker if you are in the blue alliance, according to WPILib's
	 * field coordinate system.
	 * Like last year, the field is not rotationally symmetrical, and we use the convention of "origin
	 * follows alliance", so the speaker position will be different depending on our alliance.
	 *
	 * For more information:
	 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#origin-follows-your-alliance
	 */
	val SPEAKER_POSITION_METERS_BLUE = Translation3d(0.0, 0.0, SPEAKER_HEIGHT.meters)

	/**
	 * The Translation2d of the red speaker if you are in the red alliance, according to WPILib's
	 * field coordinate system.
	 * Like last year, the field is not rotationally symmetrical, and we use the convention of "origin
	 * follows alliance", so the speaker position will be different depending on our alliance.
	 *
	 * For more information:
	 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#origin-follows-your-alliance
	 */
	val SPEAKER_POSITION_METERS_RED = Translation3d(0.0, 0.0, SPEAKER_HEIGHT.meters)


	/** Height from the floor. */
	val SHOOTER_PIVOT_HEIGHT = 0.0.centimeters


	/**
	 * The flat distance (e.g. without accounting for height difference).
	 */
	val SHOOTER_PIVOT_DISTANCE_FROM_CHASSIS_CENTER = 0.0.centimeters

	/**
	 * The 3d position of the shooter's pivot,
	 * with the floor directly under the chassis's center as the origin.
	 * Units in meters.
	 */
	val SHOOTER_PIVOT_TO_CHASSIS_CENTER_TRANSLATION3D = Translation3d(
		0.0, // X
		SHOOTER_PIVOT_DISTANCE_FROM_CHASSIS_CENTER.meters, // Y
		SHOOTER_PIVOT_HEIGHT.meters // Z
	)

	/**
	 * The shooter's angle measurement when it is parallel to the floor
	 * (or what it would have been if it could get to that position).
	 */
	val SHOOTER_PITCH_OFFSET = 0.0.degrees

	val SHOOTER_VELOCITY_SCALE_FACTOR = 0.0
}