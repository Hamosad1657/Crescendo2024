package com.hamosad1657.lib.math

import com.hamosad1657.lib.robotPrintError
import edu.wpi.first.math.MathUtil
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.floor

/**
 * If the absolute value is smaller than the deadband, the value becomes 0.
 * Otherwise, it stays the same.
 *
 * - [deadband] must be positive. [value] can be anything.
 */
fun simpleDeadband(value: Double, deadband: Double): Double {
	if (deadband < 0.0) {
		robotPrintError("deadband is negative", true)
		return value
	}
	return if (abs(value) >= deadband) value else 0.0
}

/**
 * If the absolute value is smaller than the deadband, it becomes 0.
 * Otherwise, it is mapped from a range where deadband is the minimum and 1 is the maximum,
 * to a range where zero is the minimum and 1 is the maximum. (Or, if the value is negative,
 * from a range where -1 is the minimum and -deadband is the maximum, to a range where -1 is
 * the minimum and zero is the maximum.)
 * - [deadband] must be between 0 and 1. [value] must be between -1 and 1.
 *
 * Some examples:
 * - continuousDeadband(0.05, 0.1) will return 0.0.
 * - continuousDeadband(0.1, 0.1) will return 0.0.
 * - continuousDeadband(0.5, 0.1) will return 0.44444
 * - continuousDeadband(1, 0.1) will return 1.0.
 */
fun continuousDeadband(value: Double, deadband: Double): Double {
	if (deadband !in 0.0..1.0) {
		robotPrintError("deadband is out of bounds: $deadband", true)
		return value
	}
	if (value !in -1.0..1.0) {
		robotPrintError("value is out of bounds: $value", true)
		return value
	}

	return if (value > deadband) {
		mapRange(value, deadband, 1.0, 0.0, 1.0)
	} else if (value < -deadband) {
		mapRange(value, -1.0, -deadband, -1.0, 0.0)
	} else {
		0.0
	}
}

fun clamp(value: Double, min: Double, max: Double): Double {
	return if (min > max) 0.0 else MathUtil.clamp(value, min, max)
}

/**
 * Gets a start range defined by [startMin] and [startMax] and an end range defined by [endMin] and [endMax], and a
 * value that is relative to the first range.
 *
 * @return The value relative to the end range.
 */
fun mapRange(value: Double, startMin: Double, startMax: Double, endMin: Double, endMax: Double): Double {
	if (startMin >= startMax) {
		robotPrintError("startMin is equal/bigger than starMax", true)
		return value
	}
	if (endMin >= endMax) {
		robotPrintError("endMin is equal/bigger than endMax", true)
		return value
	}
	return endMin + (endMax - endMin) / (startMax - startMin) * (value - startMin)
}

/**
 * Gets a start range defined by [startMin] and [startMax] and an end range defined by [endMin] and [endMax], and a
 * value that is relative to the first range.
 *
 * @return The value relative to the end range.
 */
fun mapRange(value: Int, startMin: Int, startMax: Int, endMin: Int, endMax: Int): Int {
	return mapRange(
		value.toDouble(),
		startMin.toDouble(),
		startMax.toDouble(),
		endMin.toDouble(),
		endMax.toDouble()
	).toInt()
}

fun median(collection: Collection<Double>): Double {
	return median(collection.toDoubleArray())
}

fun median(array: Array<Double>): Double {
	return median(array.toDoubleArray())
}

fun median(array: DoubleArray): Double {
	val sortedArray = array.sorted()
	val size = sortedArray.size
	if (size == 2) return sortedArray.average()

	return if (size % 2 == 0) {
		(sortedArray[size / 2] + sortedArray[(size / 2) - 1]) / 2.0
	} else {
		array[(size / 2)]
	}
}

/**
 * Modify the setpoint to always go the shorter way when controlling position on a circle.
 * The output of this function changes according to the measurement, so it must be updated every
 * loop iteration.
 *
 * Returns a new setpoint that will produce the shortest path to [realSetpoint], using the
 * [measurement] (which isn't required to be inside of [minPossibleSetpoint] and [maxPossibleSetpoint]).
 *
 * The [minPossibleSetpoint] and [maxPossibleSetpoint] define the range where the wrapping will occur.
 *
 * # Example
 * Say I want to control the angle of a swerve module (we'll use degrees for convenience).
 * Zero and 360 are the same position in reality, and it can cross this position with no
 * problem physically. The PID runs onboard the motor controller, not the RoboRIO.
 *
 * Now, imagine this situation: The wheel is now at 10 degrees, and the setpoint is 350.
 * Without this function, the motor will move 340 degrees all the way around, even though in
 * reality it's only 20 degrees away, because it does not know that zero and 360 are the same.
 *
 * ||||| To solve the above problem, use this function! pass 350.0 for [realSetpoint], 10.0 for
 * [measurement], 0.0 for [minPossibleSetpoint], and 360.0 for [maxPossibleSetpoint].
 * This function will return a new setpoint in degrees, which is then set to the motor controller
 * in position control mode (you convert units if needed, this function is not responsible for unit
 * conversions) and the new setpoint will make it go the shorter way.
 *
 * - Note that the measurement is allowed to accumulate beyond [minPossibleSetpoint] and
 * [maxPossibleSetpoint], but it needs to correspond to the same position in the original scope.
 * For example, a measurement of 361 must be the same module angle as measurement 1.
 *
 * * Note that the new setpoint returned is NOT "set and forget": it changes according to the
 * measurement and therefore must be updated every loop iteration. (Which you have to do anyway
 * if you use WPILib's motor safety stuff.)
 *
 * - DO NOT use this function if your mechanism cannot move freely in every direction, like
 * in a turret with finite rotation. Also don't use it for controlling linear motion, like
 * a telescopic arm or an elevator.
 *
 * * This function is not needed with a Spark Max motor controller, because it already does this.
 * It's also not needed with WPILib's PIDController, since it has ```enableContinuousInput().```
 *
 */
fun wrapPositionSetpoint(
	realSetpoint: Double,
	measurement: Double,
	minPossibleSetpoint: Double,
	maxPossibleSetpoint: Double,
): Double {
	if (!(minPossibleSetpoint < maxPossibleSetpoint)) {
		robotPrintError("minPossibleSetpoint is equal/bigger than maxPossibleSetpoint")
		return realSetpoint
	}
	if (realSetpoint !in minPossibleSetpoint..maxPossibleSetpoint) {
		robotPrintError("realSetpoint is out of bounds: $realSetpoint", true)
		return realSetpoint
	}

	val countsInRotation = maxPossibleSetpoint - minPossibleSetpoint

	// This is done in case the measurement doesn't wrap already (e.g. is accumulated forever, and could theoretically be infinite)
	val wrappedMeasurement = MathUtil.inputModulus(measurement, minPossibleSetpoint, maxPossibleSetpoint)

	val realError = realSetpoint - wrappedMeasurement
	val maxRealError = maxPossibleSetpoint - minPossibleSetpoint

	val minModifiedError = maxRealError / -2.0
	val maxModifiedError = maxRealError / 2.0

	val modifiedError = MathUtil.inputModulus(realError, minModifiedError, maxModifiedError)

	val modifiedSetpoint = modifiedError + wrappedMeasurement // same as [error = setpoint - measurement]

	val fullRotationsFromZero =
		if (measurement < minPossibleSetpoint) ceil(measurement / countsInRotation) else floor(measurement / countsInRotation)
	return modifiedSetpoint + fullRotationsFromZero * countsInRotation
}