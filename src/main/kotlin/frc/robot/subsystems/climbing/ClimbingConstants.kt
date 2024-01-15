package frc.robot.subsystems.climbing

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs
import com.ctre.phoenix6.signals.ForwardLimitSourceValue
import com.ctre.phoenix6.signals.ForwardLimitTypeValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.rps

object ClimbingConstants {
    // TODO: Tune PID
    val PID_GAINS = PIDGains(0.0, 0.0, 0.0)

    const val FULLY_CLOSED_ROTATIONS_COUNT = 0.0;
    const val FULLY_OPEN_ROTATIONS_COUNT = 0.0

    val FALCON_HARDWARE_LIMITS_CONFIG = HardwareLimitSwitchConfigs().apply {
        ForwardLimitEnable = true
        ForwardLimitAutosetPositionEnable = true
        ForwardLimitAutosetPositionValue = FULLY_CLOSED_ROTATIONS_COUNT
        ForwardLimitType = ForwardLimitTypeValue.NormallyOpen
        ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin
    }


    enum class ClimbingMode(val speedLimit: AngularVelocity, setpoint: Double) {
        REACHING_CHAIN(0.0.rps, 0.0),
        CLIMBING_UP(0.0.rps, 0.0),
        CLIMBING_DOWN(0.0.rps, 0.0),
        STAYING_FOLDED(0.0.rps, 0.0)
    }
}
