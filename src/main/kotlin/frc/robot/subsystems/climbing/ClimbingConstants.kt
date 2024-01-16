package frc.robot.subsystems.climbing

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.signals.ForwardLimitSourceValue
import com.ctre.phoenix6.signals.ForwardLimitTypeValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Rotations
import com.hamosad1657.lib.units.rps

object ClimbingConstants {
    // TODO: Tune PID
    val PID_GAINS = PIDGains(0.0, 0.0, 0.0)

    const val SETPOINT_TOLERANCE: Rotations = 0.0

    val FALCON_HARDWARE_LIMITS_CONFIG = HardwareLimitSwitchConfigs().apply {
        ForwardLimitEnable = true
        ForwardLimitAutosetPositionEnable = true
        ForwardLimitAutosetPositionValue = ClimbingState.REACHING_CHAIN.setpoint
        ForwardLimitType = ForwardLimitTypeValue.NormallyOpen
        ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin
    }

    val MOTION_MAGIC_CONFIG = MotionMagicConfigs().apply {
        MotionMagicCruiseVelocity = 0.0
        MotionMagicAcceleration = 0.0
    }

    enum class ClimbingState(val setpoint: Rotations, val voltageFF: Double, val maxVelocity: AngularVelocity) {
        REACHING_CHAIN(
            setpoint = 0.0,
            voltageFF = 0.0,
            maxVelocity = 0.0.rps,
        ),
        CLIMBING_UP(
            setpoint = 0.0,
            voltageFF = 0.0,
            maxVelocity = 0.0.rps,
        ),
        CLIMBING_DOWN(
            setpoint = 0.0,
            voltageFF = 0.0,
            maxVelocity = 0.0.rps,
        ),
        STAYING_FOLDED(
            setpoint = 0.0,
            voltageFF = 0.0,
            maxVelocity = 0.0.rps,
        ),
    }
}
