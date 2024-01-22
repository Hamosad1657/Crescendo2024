package frc.robot.subsystems.shooter

import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Rotation2d

object ShooterConstants {
    // TODO: Find velocity and angle tolerances for shooter
    val VELOCITY_TOLERANCE: AngularVelocity = 0.0.rpm
    val ANGLE_TOLERANCE = Rotation2d()

    val ANGLE_FOR_INTAKE = Rotation2d() // TODO: Find SHOOTER_ANGLE_FOR_INTAKE

    /**
     * Time between when loading started to when the note is shot.
     * It might be a little different in different speeds, so put here
     * it's maximum value.
     */
    const val SHOOT_TIME_SEC = 0.0 // TODO: Measure SHOOT_TIME_SEC

    /** 0 should be the starting position (the lowest possible angle). */
    const val CANCODER_OFFSET_DEG = 0.0

    // ShooterState is a data class and not an enum, because we might want to make
    // a continuous function (robot pose3d to target pose3d) if we have the time.
    // In the meantime, we will shoot from a few constant positions. Keep instances
    // of ShooterState as constants.
    data class ShooterState(val angle: Rotation2d, val velocity: AngularVelocity) {

        companion object {
            // TODO: Test and find the shooter states
            val TO_AMP = ShooterState(Rotation2d(), 0.0.rpm)
            val TO_TRAP = ShooterState(Rotation2d(), 0.0.rpm)

            // TODO: Name these shooter states better
            val TO_SPEAKER_1 = ShooterState(Rotation2d(), 0.0.rpm)
            val TO_SPEAKER_2 = ShooterState(Rotation2d(), 0.0.rpm)
            val TO_SPEAKER_3 = ShooterState(Rotation2d(), 0.0.rpm)

            val COLLECT = ShooterState(Rotation2d(), 0.0.rpm)
        }
    }

}