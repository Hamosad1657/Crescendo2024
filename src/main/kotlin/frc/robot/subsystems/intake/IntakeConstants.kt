package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration

object IntakeConstants {
	const val output = 0.0 // TODO: Do tests and decide optimal intake output
	val supplyCurrentLimitConfiguration =
		SupplyCurrentLimitConfiguration(true, 40.0, 60.0, 0.01) // TODO: decide optimal intake current limit configs
}