package frc.robot.subsystems.climbing

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.signals.ForwardLimitSourceValue
import com.ctre.phoenix6.signals.ForwardLimitTypeValue
import com.hamosad1657.lib.math.PIDGains

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

    const val MAX_VELOCITY = 0.0;
    val MOTION_MAGIC_CONFIG = MotionMagicConfigs().apply {
        MotionMagicCruiseVelocity = MAX_VELOCITY
    }
}
