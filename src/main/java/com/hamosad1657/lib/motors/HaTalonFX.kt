package com.hamosad1657.lib.motors

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.math.clamp
import java.lang.IllegalArgumentException

/**
 * Max safe temperature for the time span of a match.
 * This number is an educated assumption based on things I found on the internet.
 * https://www.chiefdelphi.com/uploads/short-url/eVYO5tVOYZecwq6Tl2kURlFZFgq.pdf
 * https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/
 *
 */
const val FalconSafeTempC = 90

class HaTalonFX(deviceNumber: Int) : TalonFX(deviceNumber) {
	init {
		isSafetyEnabled = true
	}

	val configuration = TalonFXConfiguration()
	val closedLoopGeneralConfigs = ClosedLoopGeneralConfigs()

	var forwardLimit: () -> Boolean = { false }
	var reverseLimit: () -> Boolean = { false }

	private var speed = 0.0


	fun configPIDGains(gains: PIDGains, slotIndex: Int = 0) {
		require(slotIndex in 0..2)

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
			else -> throw IllegalArgumentException()
		}
		configurator.apply(configuration)
	}

	override fun get() = speed


	var positionWrapEnabled = false
		set(value) {
			closedLoopGeneralConfigs.ContinuousWrap = value
			configurator.apply(closedLoopGeneralConfigs)
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


	override fun set(percentOutput: Double) {
		require(maxPercentOutput >= minPercentOutput)
		if ((forwardLimit() && percentOutput > 0.0) || (reverseLimit() && percentOutput < 0.0)) {
			speed = 0.0
			super.set(0.0)
		} else {
			speed = clamp(percentOutput, minPercentOutput, maxPercentOutput)
			super.set(speed)
		}
	}
}