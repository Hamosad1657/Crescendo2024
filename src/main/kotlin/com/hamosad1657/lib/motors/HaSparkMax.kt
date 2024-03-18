package com.hamosad1657.lib.motors

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.robotPrintError
import com.hamosad1657.lib.units.PercentOutput
import com.revrobotics.CANSparkMax
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder

/**
 * Max safe temperature for the time span of a match.
 * This number is an educated assumption based on things I found on the internet.
 * https://www.chiefdelphi.com/uploads/short-url/eVYO5tVOYZecwq6Tl2kURlFZFgq.pdf
 * https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/
 *
 */
const val NEOSafeTempC = 90

class HaSparkMax(
	deviceID: Int,
	motorType: MotorType = MotorType.kBrushless,
) : CANSparkMax(deviceID, motorType), Sendable {
	/**
	 * Software forward limit, ONLY for percent-output control.
	 * WILL NOT work in closed-loop control onboard the motor controller, since that
	 * uses SparkMaxPIDController, which is package-private and cannot be extended by us.
	 *
	 * - If possible, use hardware limits by wiring switches to the data port.
	 */
	var forwardLimit: () -> Boolean = { false }

	/**
	 * Software forward limit, ONLY for percent-output control.
	 * WILL NOT work in closed-loop control onboard the motor controller, since that
	 * uses SparkMaxPIDController, which is package-private and cannot be extended by us.
	 *
	 * - If possible, use hardware limits by wiring switches to the data port.
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

	/** The NEO motor has a temperature sensor inside it.*/
	val isMotorTempSafe get() = motorTemperature < NEOSafeTempC

	fun configPID(gains: PIDGains) {
		pidController.apply {
			p = gains.kP
			i = gains.kI
			d = gains.kD
			iZone = gains.kIZone
		}
	}

	/**
	 * percentOutput is clamped between properties minPercentOutput and maxPercentOutput.
	 */
	override fun set(output: PercentOutput) {
		if (maxPercentOutput <= minPercentOutput) {
			robotPrintError("maxPercentOutput is smaller or equal to minPercentOutput")
		} else {
			super.set(clamp(output, minPercentOutput, maxPercentOutput))
		}
	}

	fun setWithLimits(output: PercentOutput) {
		if ((forwardLimit() && output > 0.0) || (reverseLimit() && output < 0.0)) {
			super.set(0.0)
		} else {
			set(output)
		}
	}

	override fun initSendable(builder: SendableBuilder?) {
		if (builder != null) {
			builder.setSmartDashboardType("Motor Controller")
			builder.setActuator(true)
			builder.setSafeState { stopMotor() }
			builder.addDoubleProperty("Value", this::get, this::set)
		}
	}
}