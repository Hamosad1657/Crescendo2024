package frc.robot.subsystems.shooter.dynamicshooting

import com.hamosad1657.lib.units.centimeters
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.feet
import edu.wpi.first.math.geometry.Translation2d

object DynamicShootingConstants {

	val SPEAKER_HEIGHT = 0.0.feet

	/**
	 * The Translation2d of the blue speaker if you are in the blue alliance, according to WPILib's
	 * field coordinate system.
	 * Like last year, the field is not rotationally symmetrical, and we use the convention of "origin
	 * follows alliance", so the speaker position will be different depending on our alliance.
	 *
	 * For more information:
	 * [coordinate-system.html#origin-follows-your-alliance] (https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#origin-follows-your-alliance)
	 */
	val SPEAKER_POSITION_METERS_BLUE = Translation2d(0.0, 0.0)

	/**
	 * The Translation2d of the red speaker if you are in the red alliance, according to WPILib's
	 * field coordinate system.
	 * Like last year, the field is not rotationally symmetrical, and we use the convention of "origin
	 * follows alliance", so the speaker position will be different depending on our alliance.
	 *
	 * For more information:
	 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#origin-follows-your-alliance
	 */
	val SPEAKER_POSITION_METERS_RED = Translation2d(0.0, 0.0)

	/**
	 * This value will change according to the shooter's angle. I assume the difference is not
	 * substantial, so this constant contains the average value. If it does matter, create a function
	 * to calculate it instead.
	 */
	val SHOOTER_END_HEIGHT = 0.0.centimeters

	/**
	 * The flat distance (e.g. ignoring height).
	 *
	 * This value will change according to the shooter's angle. I assume the difference is not
	 * substantial, so this constant contains the average value. If it does matter, create a function
	 * to calculate it instead.
	 */
	val SHOOTER_END_FROM_ROBOT_CENTER_DISTANCE = 0.0.centimeters

	/**
	 * The shooter's angle measurement when it is parallel to the floor
	 * (or what it would have been if it could get to that position).
	 *
	 * Parallel meaning, the pivot and the
	 */
	val SHOOTER_PITCH_OFFSET = 0.0.degrees

	val SHOOTER_VELOCITY_SCALE_FACTOR = 0.0
}