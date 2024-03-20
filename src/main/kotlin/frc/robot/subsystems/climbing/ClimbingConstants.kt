package frc.robot.subsystems.climbing

import com.hamosad1657.lib.units.Amps
import com.hamosad1657.lib.units.PercentOutput

object ClimbingConstants {
	const val SMART_CURRENT_LIMIT: Amps = 40

	const val OPEN_CLIMBING_OUTPUT: PercentOutput = 0.8
	const val CLOSE_CLIMBING_OUTPUT: PercentOutput = 0.5
}
