package com.hamosad1657.lib.units

import com.hamosad1657.lib.robotPrintError

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
	private var meters = 0.0
		set(value) {
			field = if (value.isNaN()) {
				robotPrintError("Length is NaN.", true)
				0.0
			} else if (value.isInfinite()) {
				robotPrintError("Length is infinite.", true)
				0.0
			} else if (value < 0.0) {
				robotPrintError("Length cannot be negative.", true)
				0.0
			} else value
		}

	val asMeters get() = meters
	val asCentimeters get() = this.inUnit(Length.Unit.Centimeters)
	val asMillimeters get() = this.inUnit(Length.Unit.Millimeters)
	val asFeet get() = this.inUnit(Length.Unit.Feet)
	val asInches get() = this.inUnit(Length.Unit.Inches)

	init {
		meters = when (lengthUnit) {
			Length.Unit.Meters -> length.toDouble()
			Length.Unit.Centimeters -> length.toDouble() / 100
			Length.Unit.Millimeters -> length.toDouble() / 1000
			Length.Unit.Feet -> feetToMeters(length)
			Length.Unit.Inches -> inchesToMeters(length)
		}
	}

	constructor(
		meters: Number = 0,
		centimeters: Number = 0,
		millimeters: Number = 0,
		feet: Number = 0,
		inches: Number = 0,
	) : this(0.0, Length.Unit.Meters) {
		this.meters = meters.toDouble() +
			(centimeters.toDouble() * 100) +
			(millimeters.toDouble() * 1000) +
			feetToMeters(feet) +
			inchesToMeters(inches)
	}

	private fun inUnit(lengthUnit: Unit) =
		when (lengthUnit) {
			Length.Unit.Meters -> meters
			Length.Unit.Centimeters -> meters * 100.0
			Length.Unit.Millimeters -> meters * 1000.0
			Length.Unit.Feet -> metersToFeet(meters)
			Length.Unit.Inches -> metersToInches(meters)
		}

	override fun toString() = "Meters($meters)"
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
		fun fromMeters(meters: Number) = Length(meters, Length.Unit.Meters)
		fun fromCentimeters(centimeters: Number) = Length(centimeters, Length.Unit.Centimeters)
		fun fromMillimeters(millimeters: Number) = Length(millimeters, Length.Unit.Millimeters)
		fun fromFeet(feet: Number) = Length(feet, Length.Unit.Feet)
		fun fromInches(inches: Number) = Length(inches, Length.Unit.Inches)
	}
}
