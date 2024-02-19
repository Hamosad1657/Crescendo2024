package frc.robot.subsystems.loader

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.hamosad1657.lib.units.Volts

object LoaderConstants {
	const val MOTOR_INTAKE_VOLTAGE: Volts = 3.0
	const val MOTOR_LOADING_VOLTAGE: Volts = 5.0
	const val EJECT_INTO_AMP: Volts = -12.0

	const val ANALOG_READ_NOTE_DETECTED_THRESHOLD = 3530.0

	val MOTORS_CURRENT_LIMIT = CurrentLimitsConfigs().apply {
		SupplyCurrentLimitEnable = true
		SupplyCurrentLimit = 25.0
	}
}
