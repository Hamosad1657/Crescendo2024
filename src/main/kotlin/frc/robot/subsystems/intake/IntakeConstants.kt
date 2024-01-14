package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration

object IntakeConstants {
	const val OUTPUT = 0.0 // TODO: Do tests and decide optimal intake output
	val SUPPLY_CURRENT_LIMIT =
		SupplyCurrentLimitConfiguration(true, 30.0, 35.0, 0.05) // TODO: decide optimal intake current limit configs
}