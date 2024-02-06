package frc.robot.subsystems.climbing

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.FractionalOutput
import com.hamosad1657.lib.units.Rotations

object ClimbingConstants {
	// TODO: Tune PID
	// Don't use iZone it's weird with WPILib's PID controller
	val WEIGHT_BEARING_PID_GAINS = PIDGains(0.0, 0.0, 0.0, { 0.0 })
	val NO_WEIGHT_BEARING_PID_GAINS = PIDGains(0.0, 0.0, 0.0, { 0.0 })

	const val SETPOINT_TOLERANCE: Rotations = 0.0

	const val MIN_POSSIBLE_POSITION: Rotations = 0.0
	const val MAX_POSSIBLE_POSITION: Rotations = 0.0

	const val STAY_FOLDED_OUTPUT: FractionalOutput = 0.0

	enum class ClimbingState(val setpoint: Rotations, val output: FractionalOutput, val bearingWeight: Boolean) {
		REACHING_CHAIN(
			setpoint = 0.0,
			output = 0.0,
			bearingWeight = false
		),
		PULLING_UP_ROBOT(
			setpoint = 0.0,
			output = 0.0,
			bearingWeight = true
		),
		CLIMBING_DOWN(
			setpoint = 0.0,
			output = 0.0,
			bearingWeight = true
		),
		FOLDING(
			setpoint = 0.0,
			output = 0.0,
			bearingWeight = false
		),
	}
}
