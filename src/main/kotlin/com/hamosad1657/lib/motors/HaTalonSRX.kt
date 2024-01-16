package com.hamosad1657.lib.motors

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.math.wrapPositionSetpoint

class HaTalonSRX(deviceID: Int) : WPI_TalonSRX(deviceID) {
    init {
        isSafetyEnabled = true
    }

    /**
     * Software forward limit. Only used for [ControlMode.PercentOutput].
     *
     * If possible, use hardware limits by wiring limit switches to the data port.
     */
    var forwardLimit: () -> Boolean = { false }

    /**
     * Software reverse limit. Only used for [ControlMode.PercentOutput].
     *
     * If possible, use hardware limits by wiring limit switches to the data port.
     */
    var reverseLimit: () -> Boolean = { false }

    var minPercentOutput = -1.0
        set(value) {
            field = value.coerceAtLeast(-1.0)
        }
    var maxPercentOutput = 1.0
        set(value) {
            field = value.coerceAtMost(1.0)
        }

    private var minPositionSetpoint: Double = 0.0
    private var maxPositionSetpoint: Double = 0.0
    private var isPositionWrapEnabled = false
    private var speed = 0.0

    fun configPIDGains(gains: PIDGains, slotIndex: Int = 0) {
        require(slotIndex in 0..2)
        config_kP(slotIndex, gains.kP)
        config_kI(slotIndex, gains.kI)
        config_kD(slotIndex, gains.kD)
        config_IntegralZone(0, gains.iZone)
    }

    /**
     * Common interface for getting the current set speed of a speed controller.
     *
     * @return The current set speed. Value is between -1.0 and 1.0.
     */
    override fun get() = speed

    /**
     * In PercentOutput control mode, value is clamped between [minPercentOutput] and [maxPercentOutput].
     *
     * In Position control mode, if [isPositionWrapEnabled] is true,
     * the position is wrapped using [wrapPositionSetpoint].
     */
    override fun set(mode: ControlMode, value: Double) {
        if (mode == ControlMode.PercentOutput) {
            this.set(value)
        } else if (mode == ControlMode.Position && isPositionWrapEnabled) {
            val newValue =
                wrapPositionSetpoint(value, selectedSensorPosition, minPositionSetpoint, maxPositionSetpoint)
            super.set(ControlMode.Position, newValue)
        } else {
            super.set(mode, value)
        }
    }

    /**
     * [percentOutput] is clamped between [minPercentOutput] and [maxPercentOutput].
     */
    override fun set(percentOutput: Double) {
        require(maxPercentOutput >= minPercentOutput)
        if ((forwardLimit() && percentOutput > 0.0) || (reverseLimit() && percentOutput < 0.0)) {
            speed = 0.0
            super.set(ControlMode.PercentOutput, 0.0)
        } else {
            speed = clamp(percentOutput, minPercentOutput, maxPercentOutput)
            super.set(ControlMode.PercentOutput, speed)
        }
    }

    /**
     * "Position wrap" means always going the shorter way. For example, if the current
     * position is 10 degrees and the setpoint is 350 degrees, then with position wrap
     * it would just move 20 degrees to the setpoint (while without position wrap it
     * would go all the way around).
     *
     * - For more information, see [com.hamosad1657.lib.math.wrapPositionSetpoint].
     *
     * @param minPossibleSetpoint The smallest setpoint.
     * @param maxPossibleSetpoint The largest setpoint.
     */
    fun enablePositionWrap(minPossibleSetpoint: Double, maxPossibleSetpoint: Double) {
        require(minPossibleSetpoint < maxPossibleSetpoint)
        this.minPositionSetpoint = minPossibleSetpoint
        this.maxPositionSetpoint = maxPossibleSetpoint
        isPositionWrapEnabled = true
    }

    fun disablePositionWrap() {
        isPositionWrapEnabled = false
    }
}