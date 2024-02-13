package frc.robot.subsystems.climbing

import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Rotations

object ClimbingConstants {
	// TODO: Tune PID
	// Don't use iZone it's weird with WPILib's PID controller
	val PID_GAINS_HOLDING_ROBOT = PIDGains(0.0, 0.0, 0.0, { 0.0 })
	val PID_GAINS_NOT_HOLDING_ROBOT = PIDGains(0.0, 0.0, 0.0, { 0.0 })

	const val SETPOINT_TOLERANCE: Rotations = 0.0

	const val MAX_POSSIBLE_POSITION: Rotations = 0.0

	const val STAY_FOLDED_OUTPUT: PercentOutput = 0.0

	enum class ClimbingState(val setpoint: Rotations, val output: PercentOutput, val isHoldingRobot: Boolean) {
		REACHING_CHAIN(
			setpoint = 0.0,
			output = 0.0,
			isHoldingRobot = false
		),
		PULLING_UP_ROBOT(
			setpoint = 0.0,
			output = 0.0,
			isHoldingRobot = true
		),
		CLIMBING_DOWN(
			setpoint = 0.0,
			output = 0.0,
			isHoldingRobot = true
		),
		FOLDING(
			setpoint = 0.0,
			output = 0.0,
			isHoldingRobot = false
		),
	}
}
