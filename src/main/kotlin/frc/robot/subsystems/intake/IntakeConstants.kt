package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration

object IntakeConstants {
	// TODO: Do tests and decide optimal intake output
	const val OUTPUT = 0.0

	// TODO: decide optimal intake current limit configs
	val SUPPLY_CURRENT_LIMIT = SupplyCurrentLimitConfiguration(true, 30.0, 35.0, 0.05)
}