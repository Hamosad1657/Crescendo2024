package frc.robot.subsystems.stabilizers

import com.hamosad1657.lib.units.Amps
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Volts

object StabilizersConstants {
	const val SMART_CURRENT_LIMIT: Amps = 20

	const val OPEN_STABILIZERS_OUTPUT: Volts = 6.0
	const val CLOSE_STABILIZERS_OUTPUT: PercentOutput = -6.0

	const val MOTOR_STALL_CURRENT: Amps = 15
}