package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.hamosad1657.lib.units.Volts

object IntakeConstants {
	const val BOTTOM_MOTOR_VOLTAGE: Volts = 5.0
	const val TOP_MOTOR_VOLTAGE: Volts = 12.0

	val MOTORS_CURRENT_LIMIT = CurrentLimitsConfigs().apply {
		SupplyCurrentLimitEnable = true
		SupplyCurrentLimit = 40.0
	}
}
