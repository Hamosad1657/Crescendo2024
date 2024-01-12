package com.hamosad1657.lib.units

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
private constructor(velocity: Double, velocityUnit: AngularVelocity.Unit) : Comparable<AngularVelocity> {
	var rpm = 0.0
		set(value) {
			require(!value.isNaN())
			require(value.isFinite())
			field = value
		}

	var rps = this.inUnit(AngularVelocity.Unit.Rps)
		get() = this.inUnit(AngularVelocity.Unit.Rps)
		set(value) {
			rpm = rpsToRpm(value)
			field = value
		}

	var radPs = this.inUnit(AngularVelocity.Unit.RadPs)
		get() = this.inUnit(AngularVelocity.Unit.RadPs)
		set(value) {
			rpm = radPsToRpm(value)
			field = value
		}

	var degPs = this.inUnit(AngularVelocity.Unit.DegPs)
		get() = this.inUnit(AngularVelocity.Unit.DegPs)
		set(value) {
			rpm = degPsToRpm(value)
			field = value
		}

	init {
		rpm = when (velocityUnit) {
			AngularVelocity.Unit.Rpm -> velocity
			AngularVelocity.Unit.Rps -> rpsToRpm(velocity)
			AngularVelocity.Unit.RadPs -> radPsToRpm(velocity)
			AngularVelocity.Unit.DegPs -> degPsToRpm(velocity)
		}
	}

	private fun inUnit(velocityUnit: AngularVelocity.Unit) =
		when (velocityUnit) {
			Unit.Rpm -> rpm
			Unit.Rps -> rpmToRps(rpm)
			Unit.RadPs -> rpmToRadPs(rpm)
			Unit.DegPs -> rpmToDegPs(rpm)
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
		fun fromRpm(rpm: Double) = AngularVelocity(rpm, AngularVelocity.Unit.Rpm)
		fun fromRps(rps: Double) = AngularVelocity(rps, AngularVelocity.Unit.Rps)
		fun fromRadPs(radPs: Double) = AngularVelocity(radPs, AngularVelocity.Unit.RadPs)
		fun fromDegPs(degPs: Double) = AngularVelocity(degPs, AngularVelocity.Unit.DegPs)
		fun fromMps(mps: Double, wheelRadius: Length) = fromRpm(mpsToRpm(mps, wheelRadius))
		fun fromFalconTicksPer100ms(ticksPer100ms: Double, gearRatio: Double = 1.0) =
			fromRpm(falconTicksPer100msToRpm(ticksPer100ms, gearRatio))
	}
}
