package frc.robot.subsystems.shooter

import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.rpm
import edu.wpi.first.math.geometry.Rotation2d

object ShooterConstants {
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