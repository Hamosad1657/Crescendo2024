package com.hamosad1657.lib.motors

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.robotPrintError
import com.hamosad1657.lib.units.PercentOutput
import com.revrobotics.CANSparkFlex
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import kotlin.math.absoluteValue

class HaSparkFlex(
	deviceID: Int,
	motorType: MotorType = MotorType.kBrushless,
) : CANSparkFlex(deviceID, motorType), Sendable {
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

	/** If the motor's voltage output is smaller than this value, the motor will stop.  */
	var voltageNeutralDeadband = 0.0

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

	override fun setVoltage(outputVolts: Double) {
		if (outputVolts.absoluteValue < voltageNeutralDeadband) super.stopMotor()
		else super.setVoltage(outputVolts)
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