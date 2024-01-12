package com.hamosad1657.lib.math

import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile

/**
 * Contains the following gains:
 * - [kP] Proportional gain.
 * - [kI] Integral gain.
 * - [kD] Derivative gain.
 * - [kFF] Feed Forward gain.
 * - [kIZone] If the absolute error is above IZone, the integral accumulator is cleared
 * (making it ineffective). Motor controllers have this feature, but WPILib don't.
 *
 * And some methods to convert it to the following objects:
 * - [toPIDController] for conversion to a [PIDController].
 * - [toProfiledPIDController] for conversion to a [ProfiledPIDController].
 * - [toPathPlannerPIDConstants] for conversion to [PIDConstants].
 */
class PIDGains @JvmOverloads constructor(
	var kP: Double = 0.0,
	var kI: Double = 0.0,
	var kD: Double = 0.0,
	var kFF: Double = 0.0,
	var kIZone: Double = 0.0,
) {
	/** Creates a WPILib [PIDController] with the P, I and D gains. */
	fun toPIDController() = PIDController(kP, kI, kD)

	/** Creates a WPILib [ProfiledPIDController] with the P, I, and D gains and the given [constraints]. */
	fun toProfiledPIDController(constraints: TrapezoidProfile.Constraints) =
		ProfiledPIDController(kP, kI, kD, constraints)

	/** Converts the P, I and D gains to a Path Planner [PIDConstants]. */
	fun toPathPlannerPIDConstants() = PIDConstants(kP, kI, kD)
}

