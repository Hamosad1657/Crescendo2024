package com.hamosad1657.lib.motors

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.robotPrintError
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.toNeutralModeValue
import com.revrobotics.CANSparkBase.IdleMode

/**
 * Max safe temperature for the time span of a match.
 * This number is an educated assumption based on things I found on the internet.
 * https://www.chiefdelphi.com/uploads/short-url/eVYO5tVOYZecwq6Tl2kURlFZFgq.pdf
 * https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/
 */
const val FalconSafeTempC = 90

class HaTalonFX(deviceNumber: Int) : TalonFX(deviceNumber) {
	init {
		isSafetyEnabled = true
	}

	fun restoreFactoryDefaults() {
		configurator.apply(TalonFXConfiguration())
	}

	/**
	 * Sets the [NeutralModeValue] of the motor.
	 *
	 * Use only as setter.
	 */
	var idleMode: IdleMode = IdleMode.kBrake
		get() {
			robotPrintError("Use [idleMode] field only as a setter", true)
			return field
		}
		set(value) {
			super.setNeutralMode(value.toNeutralModeValue())
			field = value
		}

	/** USE [idleMode] SETTER INSTEAD. */
	override fun setNeutralMode(neutralMode: NeutralModeValue) {
		robotPrintError("Use [idleMode] setter instead of [setNeutralMode]", true)
	}

	var forwardLimit: () -> Boolean = { false }
	var reverseLimit: () -> Boolean = { false }

	private var speed = 0.0

	fun configPID(gains: PIDGains, slotIndex: Int = 0) {
		require(slotIndex in 0..2)
		val configuration = TalonFXConfiguration().apply(configurator::refresh)
		when (slotIndex) {
			0 -> configuration.Slot0.apply {
				kP = gains.kP
				kI = gains.kI
				kD = gains.kD
			}

			1 -> configuration.Slot1.apply {
				kP = gains.kP
				kI = gains.kI
				kD = gains.kD
			}

			2 -> configuration.Slot2.apply {
				kP = gains.kP
				kI = gains.kI
				kD = gains.kD
			}

			else -> robotPrintError("Illegal PID Slot: $slotIndex")
		}
		configurator.apply(configuration)
	}

	var positionWrapEnabled = false
		set(value) {
			val configuration = ClosedLoopGeneralConfigs().apply(configurator::refresh)
			configuration.ContinuousWrap = value
			configurator.apply(configuration)
			field = value
		}

	/**
	 * For use with set(percentOutput: Double)
	 */
	var minPercentOutput = -1.0
		set(value) {
			field = value.coerceAtLeast(-1.0)
		}

	/**
	 * For use with set(percentOutput: Double)
	 */
	var maxPercentOutput = 1.0
		set(value) {
			field = value.coerceAtMost(1.0)
		}

	val isTempSafe: Boolean
		get() = deviceTemp.value < FalconSafeTempC

	override fun set(output: PercentOutput) {
		if (maxPercentOutput > minPercentOutput) {
			speed = clamp(output, minPercentOutput, maxPercentOutput)
			super.set(speed)
		} else {
			robotPrintError("maxPercentOutput is smaller than minPercentOutput")
		}
	}

	fun setWithLimits(output: PercentOutput) {
		if ((forwardLimit() && output > 0.0) || (reverseLimit() && output < 0.0)) {
			speed = 0.0
			super.set(0.0)
		} else {
			set(output)
		}
	}
}