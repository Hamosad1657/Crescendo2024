package frc.robot.subsystems.loader

import com.hamosad1657.lib.units.PercentOutput

object LoaderConstants {
	/**
	 * Time that loader should be run to get the note from the intake,
	 * without putting it in the shooter yet.
	 */
	const val LOAD_FROM_INTAKE_TIME_SEC = 0.0 // TODO: Measure LOAD_FROM_INTAKE_TIME_SEC

	const val MOTOR_OUTPUT: PercentOutput = 0.9

	const val ANALOG_READ_NOTE_DETECTED_THRESHOLD = 3400.0
}