package frc.robot.subsystems.shooter.dynamicshooting

import com.hamosad1657.lib.units.*
import edu.wpi.first.math.geometry.Translation2d

object DynamicShootingConstants {
	val SPEAKER_HEIGHT = 0.feet

	/**
	 * The Translation2d of the blue speaker if you are in the blue alliance, according to WPILib's
	 * field coordinate system.
	 * Like last year, the field is not rotationally symmetrical, and we use the convention of "origin
	 * follows alliance", so the speaker position will be different depending on our alliance.
	 *
	 * For more information:
	 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#origin-follows-your-alliance
	 */
	val SPEAKER_BLUE_POSITION_METERS = Translation2d(0.0, 0.0)

	/**
	 * The Translation2d of the red speaker if you are in the red alliance, according to WPILib's
	 * field coordinate system.
	 * Like last year, the field is not rotationally symmetrical, and we use the convention of "origin
	 * follows alliance", so the speaker position will be different depending on our alliance.
	 *
	 * For more information:
	 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#origin-follows-your-alliance
	 */
	val SPEAKER_RED_POSITION_METERS = Translation2d(0.0, 0.0)

	/**
	 * The flat 2d position of the shooter's pivot (e.g. without accounting for height difference),
	 * with the floor directly under the chassis's center as the origin.
	 */
	val SHOOTER_PIVOT_TO_CHASSIS_CENTER = Translation2d(0.0, 0.0)

	val SHOOTER_HEIGHT = 0.centimeters

	/**
	 * The shooter's angle measurement when it is parallel to the floor
	 * (or what it would have been if it could get to that position).
	 */
	val SHOOTER_ANGLE_OFFSET = 0.degrees

	const val SHOOTER_VELOCITY_SCALE_FACTOR = 0.0
}