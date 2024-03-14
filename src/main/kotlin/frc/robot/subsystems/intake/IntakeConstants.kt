package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.hamosad1657.lib.units.Amps
import com.hamosad1657.lib.units.Seconds
import com.hamosad1657.lib.units.Volts

object IntakeConstants {
	val CURRENT_LIMITS_CONFIGS =
		CurrentLimitsConfigs().apply {
			SupplyCurrentLimitEnable = true
			SupplyCurrentLimit = 40.0
		}

	// TODO: Test and find actual value
	const val BOTTOM_MOTOR_UNDER_LOAD_THRESHOLD: Amps = 20

	const val BOTTOM_MOTOR_OUTPUT: Volts = 12.0
	const val TOP_MOTOR_OUTPUT: Volts = 12.0

	const val EJECT_DURATION: Seconds = 3.0
}
