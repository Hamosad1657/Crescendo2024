package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.CurrentLimitsConfigs

object IntakeConstants {
	// TODO: decide optimal intake current limit configs
	val CURRENT_LIMIT_CONFIGURATION = CurrentLimitsConfigs().apply {
		SupplyCurrentLimit = 0.0
		StatorCurrentLimit = 0.0
		SupplyCurrentLimitEnable = true
		StatorCurrentLimitEnable = true
	}

	// TODO: Do tests and decide optimal intake output
	const val MOTOR_OUTPUT = 0.0
}