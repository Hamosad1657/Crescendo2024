package frc.robot.subsystems.stabilizers

import com.hamosad1657.lib.units.Amps
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Seconds
import com.hamosad1657.lib.units.Volts

object StabilizersConstants {
	const val SMART_CURRENT_LIMIT: Amps = 30

	const val OPEN_STABILIZERS_OUTPUT: Volts = 1.0
	const val CLOSE_STABILIZERS_OUTPUT: PercentOutput = -1.0

	const val OPEN_CLOSE_DURATION: Seconds = 0.3
}