package frc.robot.subsystems.loader

import com.hamosad1657.lib.units.Volts

object LoaderConstants {
	/**
	 * Time that loader should be run to get the note from the intake,
	 * without putting it in the shooter yet.
	 */
	const val LOAD_FROM_INTAKE_TIME_SEC = 0.0 // TODO: Measure LOAD_FROM_INTAKE_TIME_SEC

	const val MOTOR_INTAKE_VOLTAGE: Volts = 2.0
	const val MOTOR_LOADING_VOLTAGE: Volts = 4.0

	const val ANALOG_READ_NOTE_DETECTED_THRESHOLD = 3530.0
}