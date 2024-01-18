package frc.robot.subsystems.shooter

import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.math.geometry.Rotation2d

object ShooterConstants {
    data class ShooterState(val angle: Rotation2d, val velocity: AngularVelocity)

}