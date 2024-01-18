package frc.robot.subsystems.shooter

import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Rotation2d

object ShooterConstants {
    val SHOOTING_VELOCITY_TOLERANCE: AngularVelocity = 0.0.rpm
    val SHOOTING_ANGLE_TOLERANCE = Rotation2d()

    val SHOOTER_ANGLE_FOR_INTAKE = Rotation2d() // TODO: Find SHOOTER_ANGLE_FOR_INTAKE

    /**
     * Time between when loading started to when the note is shot.
     * It might be a little different in different speeds, so put here
     * it's maximum value.
     */
    const val SHOOT_TIME_SEC = 0.0 // TODO: Measure SHOOT_TIME_SEC

    /**
     * Time that loader should be run to get the note from the intake,
     * without putting it in the shooter yet.
     */
    const val LOAD_FROM_INTAKE_TIME_SEC = 0.0 // TODO: Measure LOAD_FROM_INTAKE_TIME_SEC

    const val LOADER_OUTPUT = 0.0

    /**
     * Lowest possible angle should be 0.
     */
    const val CANCODER_OFFSET_DEG = 0.0

    // ShooterState is a data class and not an enum, because we might want to make
    // a continuous function (robot pose3d to target pose3d) if we have the time.
    // In the meantime, we will shoot from a few constant positions. Keep instances
    // of ShooterState as constants.
    data class ShooterState(val angle: Rotation2d, val velocity: AngularVelocity)

    // TODO: Test and find the shooter states
    val SHOOT_AMP = ShooterState(Rotation2d(), 0.0.rpm)
    val SHOOT_TRAP = ShooterState(Rotation2d(), 0.0.rpm)

    // TODO: Name these shooter states better
    val SHOOT_SPEAKER_1 = ShooterState(Rotation2d(), 0.0.rpm)
    val SHOOT_SPEAKER_2 = ShooterState(Rotation2d(), 0.0.rpm)
    val SHOOT_SPEAKER_3 = ShooterState(Rotation2d(), 0.0.rpm)
}