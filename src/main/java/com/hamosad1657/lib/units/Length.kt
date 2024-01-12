package com.hamosad1657.lib.units

/** Represents a length.
 *
 * Can be created from or converted to any of the following units:
 * - Meters
 * - Centimeters
 * - Millimeters
 * - Feet
 * - Inches
 */
class Length private constructor(length: Number, lengthUnit: Unit) : Comparable<Length> {
	var meters = 0.0
		set(value) {
			require(!value.isNaN()) { "Length cannot contain NaN." }
			require(value.isFinite()) { "Length cannot contain infinite values." }
			require(value >= 0.0) { "Length cannot be negative." }
			field = value
		}

	var centimeters = this.inUnit(Length.Unit.Centimeters)
		get() = this.inUnit(Length.Unit.Centimeters)
		set(value) {
			meters = value / 100.0
			field = value
		}

	var millimeters = this.inUnit(Length.Unit.Millimeters)
		get() = this.inUnit(Length.Unit.Millimeters)
		set(value) {
			meters = value / 1000.0
			field = value
		}

	var feet = this.inUnit(Length.Unit.Millimeters)
		get() = this.inUnit(Length.Unit.Feet)
		set(value) {
			meters = feetToMeters(value)
			field = value
		}

	var inches = this.inUnit(Length.Unit.Millimeters)
		get() = this.inUnit(Length.Unit.Inches)
		set(value) {
			meters = inchesToMeters(value)
			field = value
		}

	init {
		meters = when (lengthUnit) {
			Length.Unit.Meters -> length.toDouble()
			Length.Unit.Centimeters -> length.toDouble() / 100.0
			Length.Unit.Millimeters -> length.toDouble() / 1000.0
			Length.Unit.Feet -> feetToMeters(length)
			Length.Unit.Inches -> inchesToMeters(length)
		}
	}

	constructor(
		meters: Double = 0.0,
		centimeters: Double = 0.0,
		millimeters: Double = 0.0,
		feet: Double = 0.0,
		inches: Double = 0.0,
	) : this(0.0, Length.Unit.Meters) {
		this.meters +=
			meters + centimeters * 100.0 + millimeters * 1000.0 + feetToMeters(feet) + inchesToMeters(inches)
	}

	private fun inUnit(lengthUnit: Unit) =
		when (lengthUnit) {
			Length.Unit.Meters -> meters
			Length.Unit.Centimeters -> meters * 100.0
			Length.Unit.Millimeters -> meters * 1000.0
			Length.Unit.Feet -> metersToFeet(meters)
			Length.Unit.Inches -> metersToInches(meters)
		}

	override fun toString() = "Meters=$meters"
	override fun compareTo(other: Length) = (meters - other.meters).toInt()

	operator fun plus(other: Length) = fromMeters(meters + other.meters)
	operator fun minus(other: Length) = fromMeters(meters - other.meters)
	operator fun times(other: Double) = fromMeters(meters * other)
	operator fun div(other: Double) = fromMeters(meters / other)

	enum class Unit {
		Meters,
		Centimeters,
		Millimeters,
		Feet,
		Inches,
	}

	companion object {
		fun fromMeters(meters: Number) = Length(meters.toDouble(), Length.Unit.Meters)
		fun fromCentimeters(centimeters: Number) = Length(centimeters.toDouble(), Length.Unit.Centimeters)
		fun fromMillimeters(millimeters: Number) = Length(millimeters.toDouble(), Length.Unit.Millimeters)
		fun fromFeet(feet: Number) = Length(feet.toDouble(), Length.Unit.Feet)
		fun fromInches(inches: Number) = Length(inches.toDouble(), Length.Unit.Inches)
	}
}

