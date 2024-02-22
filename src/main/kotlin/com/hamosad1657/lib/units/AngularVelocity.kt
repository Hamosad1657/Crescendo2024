package com.hamosad1657.lib.units

import kotlin.math.absoluteValue
import com.hamosad1657.lib.units.AngularVelocity.Unit as AngularVelocityUnit

/** Represents an angular velocity.
 *
 * Can be created from or converted to any of the following units:
 * - Rotations per Minute (Rpm)
 * - Rotations per Second (Rps)
 * - Radians per Second (RadPs)
 * - Degrees per Second (DegPs)
 *
 * Can also be converted to:
 * - Meters per Second (Mps)
 * - Falcon's Integrated Encoder Ticks per 100ms
 */
class AngularVelocity
private constructor(velocity: Double, velocityUnit: AngularVelocityUnit) : Comparable<AngularVelocity> {
	private var rpm = 0.0
		set(value) {
			require(!value.isNaN()) { "AngularVelocity cannot be NaN." }
			require(value.isFinite()) { "AngularVelocity cannot be infinite." }
			field = value
		}

	val asRpm get() = rpm
	val asRps get() = this.inUnit(AngularVelocityUnit.Rps)
	val asRadPs get() = this.inUnit(AngularVelocityUnit.RadPs)
	val asDegPs get() = this.inUnit(AngularVelocityUnit.DegPs)

	fun asMps(wheelRadius: Length) = rpmToMps(rpm, wheelRadius)
	fun asFalconTicksPer100ms(gearRatio: Double = 1.0) = rpmToFalconTicksPer100ms(rpm, gearRatio)

	init {
		rpm = when (velocityUnit) {
			AngularVelocityUnit.Rpm -> velocity
			AngularVelocityUnit.Rps -> rpsToRpm(velocity)
			AngularVelocityUnit.RadPs -> radPsToRpm(velocity)
			AngularVelocityUnit.DegPs -> degPsToRpm(velocity)
		}
	}

	private fun inUnit(velocityUnit: AngularVelocityUnit) =
		when (velocityUnit) {
			AngularVelocityUnit.Rpm -> rpm
			AngularVelocityUnit.Rps -> rpmToRps(rpm)
			AngularVelocityUnit.RadPs -> rpmToRadPs(rpm)
			AngularVelocityUnit.DegPs -> rpmToDegPs(rpm)
		}

	fun abs() = fromRpm(rpm.absoluteValue)

	override fun toString() = "RPM($rpm)"
	override fun compareTo(other: AngularVelocity): Int = (rpm - other.rpm).toInt()

	operator fun plus(other: AngularVelocity) = fromRpm(rpm + other.rpm)
	operator fun minus(other: AngularVelocity) = fromRpm(rpm - other.rpm)
	operator fun times(ratio: Double) = fromRpm(rpm * ratio)
	operator fun div(ratio: Double) = fromRpm(rpm / ratio)

	enum class Unit {
		Rpm,
		Rps,
		RadPs,
		DegPs,
	}

	companion object {
		fun fromRpm(rpm: Double) = AngularVelocity(rpm, AngularVelocityUnit.Rpm)
		fun fromRps(rps: Double) = AngularVelocity(rps, AngularVelocityUnit.Rps)
		fun fromRadPs(radPs: Double) = AngularVelocity(radPs, AngularVelocityUnit.RadPs)
		fun fromDegPs(degPs: Double) = AngularVelocity(degPs, AngularVelocityUnit.DegPs)
		fun fromMps(mps: Double, wheelRadius: Length) = fromRpm(mpsToRpm(mps, wheelRadius))
		fun fromFalconTicksPer100ms(ticksPer100ms: Double, gearRatio: Double = 1.0) =
			fromRpm(falconTicksPer100msToRpm(ticksPer100ms, gearRatio))
	}
}
