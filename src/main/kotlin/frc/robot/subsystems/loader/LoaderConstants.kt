package frc.robot.subsystems.loader

import com.hamosad1657.lib.units.FractionalOutput

object LoaderConstants {
	/**
	 * Time that loader should be run to get the note from the intake,
	 * without putting it in the shooter yet.
	 */
	const val LOAD_FROM_INTAKE_TIME_SEC = 0.0 // TODO: Measure LOAD_FROM_INTAKE_TIME_SEC

	const val MOTOR_OUTPUT: FractionalOutput = 0.0

	const val ANALOG_INPUT_THRESHOLD = 1.0
}