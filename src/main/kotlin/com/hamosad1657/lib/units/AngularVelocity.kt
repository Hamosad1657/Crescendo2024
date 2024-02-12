package com.hamosad1657.lib.units

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
	var rpm = 0.0
		set(value) {
			require(!value.isNaN())
			require(value.isFinite())
			field = value
		}

	var rps = this.inUnit(AngularVelocityUnit.Rps)
		get() = this.inUnit(AngularVelocityUnit.Rps)
		set(value) {
			rpm = rpsToRpm(value)
			field = value
		}

	var radPs = this.inUnit(AngularVelocityUnit.RadPs)
		get() = this.inUnit(AngularVelocityUnit.RadPs)
		set(value) {
			rpm = radPsToRpm(value)
			field = value
		}

	var degPs = this.inUnit(AngularVelocityUnit.DegPs)
		get() = this.inUnit(AngularVelocityUnit.DegPs)
		set(value) {
			rpm = degPsToRpm(value)
			field = value
		}

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

	override fun toString() = "RPM=$rpm"
	override fun compareTo(other: AngularVelocity): Int = (rpm - other.rpm).toInt()

	operator fun plus(other: AngularVelocity) = fromRpm(rpm + other.rpm)
	operator fun minus(other: AngularVelocity) = fromRpm(rpm - other.rpm)
	operator fun times(ratio: Double) = fromRpm(rpm * ratio)
	operator fun div(ratio: Double) = fromRpm(rpm / ratio)

	fun toMps(wheelRadius: Length) = rpmToMps(rpm, wheelRadius)
	fun toFalconTicksPer100ms(gearRatio: Double = 1.0) = rpmToFalconTicksPer100ms(rpm, gearRatio)

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
