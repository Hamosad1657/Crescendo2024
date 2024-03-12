package com.hamosad1657.lib.motors

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.math.wrapPositionSetpoint
import com.hamosad1657.lib.robotPrintError
import com.hamosad1657.lib.units.toNeutralMode
import com.revrobotics.CANSparkBase.IdleMode

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

	/**
	 * Sets the [NeutralMode] of the motor.
	 *
	 * Use only as setter.
	 * */
	var idleMode: IdleMode
		get() {
			robotPrintError("Use [idleMode] field only as a setter", true)
			return IdleMode.kCoast
		}
		set(value) = setNeutralMode(idleMode.toNeutralMode())

	/** USE [idleMode] SETTER INSTEAD. */
	override fun setNeutralMode(neutralMode: NeutralMode?) {
		robotPrintError("Use [idleMode] setter instead of [setNeutralMode]", true)
	}

	private var minPositionSetpoint: Double = 0.0
	private var maxPositionSetpoint: Double = 0.0
	private var isPositionWrapEnabled = false
	private var speed = 0.0

	fun configPIDGains(gains: PIDGains, slotIndex: Int = 0) {
		if (slotIndex !in 0..3) {
			return robotPrintError("Illegal PID Slot: $slotIndex")
		}
		config_kP(slotIndex, gains.kP)
		config_kI(slotIndex, gains.kI)
		config_kD(slotIndex, gains.kD)
		config_IntegralZone(slotIndex, gains.kIZone)
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
		if ((forwardLimit() && percentOutput > 0.0) || (reverseLimit() && percentOutput < 0.0)) {
			speed = 0.0
			super.set(ControlMode.PercentOutput, 0.0)
		} else {
			if (maxPercentOutput > minPercentOutput) {
				speed = clamp(percentOutput, minPercentOutput, maxPercentOutput)
				super.set(ControlMode.PercentOutput, speed)
			} else {
				robotPrintError("maxPercentOutput is smaller then minPercentOutput", true)
			}
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
		if (!(minPossibleSetpoint < maxPossibleSetpoint)) {
			robotPrintError("minPossibleSetpoint is bigger than maxPossibleSetpoint", true)
		}
		this.minPositionSetpoint = minPossibleSetpoint
		this.maxPositionSetpoint = maxPossibleSetpoint
		isPositionWrapEnabled = true
	}

	fun disablePositionWrap() {
		isPositionWrapEnabled = false
	}
}