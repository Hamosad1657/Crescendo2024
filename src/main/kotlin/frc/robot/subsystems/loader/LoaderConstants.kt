package frc.robot.subsystems.loader

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.hamosad1657.lib.units.Seconds
import com.hamosad1657.lib.units.Volts

object LoaderConstants {
	val MOTORS_CURRENT_LIMIT =
		CurrentLimitsConfigs().apply {
			SupplyCurrentLimitEnable = true
			SupplyCurrentLimit = 15.0
		}

	const val ANALOG_READ_NOTE_DETECTED_THRESHOLD = 3550.0

	const val MOTOR_INTAKE_OUTPUT: Volts = 4.0
	const val MOTOR_LOADING_OUTPUT: Volts = 5.0
	const val MOTOR_EJECT_OUTPUT: Volts = -4.0

	/** Time between when loading started to when the note is inside of the Amp. */
	const val AMP_EJECT_DURATION: Seconds = 2.0
}
