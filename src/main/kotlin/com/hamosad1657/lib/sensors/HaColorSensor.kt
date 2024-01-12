package com.hamosad1657.lib.sensors

import com.hamosad1657.lib.math.mapRange
import com.hamosad1657.lib.units.Length
import com.hamosad1657.lib.units.centimeters
import com.revrobotics.ColorSensorV3
import com.revrobotics.ColorSensorV3.RawColor
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.util.Color

class HaColorSensor(port: I2C.Port) : Sendable {
	private val colorSensor: ColorSensorV3

	/** The raw color detected by the sensor. Contains red, green, blue and IR. */
	val colorRaw: RawColor get() = colorSensor.rawColor

	/** The red value of the raw detected color. */
	val redRaw get() = colorSensor.red

	/** The green value of the raw detected color. */
	val greenRaw get() = colorSensor.green

	/** The blue value of the raw detected color. */
	val blueRaw get() = colorSensor.blue

	/** The IR value of the raw detected color (in CIE 1931 XYZ colorspace). */
	val ir get() = colorSensor.ir

	/** The proximity value of the sensor ranging from 0 (object is close) to 2047 (object is far away). */
	val proximityRaw get() = MAX_PROXIMITY - colorSensor.proximity // Flip the range from [2047, 0] to [0, 2047]

	/** The proximity of the sensor. */
	val proximityCentimeters: Length
		get() = mapRange(proximityRaw, MIN_PROXIMITY, MAX_PROXIMITY, MIN_DISTANCE_CM, MAX_DISTANCE_CM).centimeters

	/** The detected color, normalized to percents. */
	val color: Color get() = colorSensor.color

	/** The detected red color, normalized to percents with green and blue. */
	val red get() = colorSensor.color.red

	/** The detected green color, normalized to percents with red and blue. */
	val green get() = colorSensor.color.green

	/** The detected blue color, normalized to percents with red and green. */
	val blue get() = colorSensor.color.blue

	init {
		colorSensor = ColorSensorV3(port)
	}

	/** Checks whether the raw detected color is in the specified range (inclusive). */
	fun isRawColorInRange(minColorRaw: RawColor, maxColorRaw: RawColor): Boolean {
		val color = colorSensor.rawColor
		return color.red >= minColorRaw.red &&
				color.red <= maxColorRaw.red &&
				color.blue >= minColorRaw.blue &&
				color.blue <= maxColorRaw.blue &&
				color.green >= minColorRaw.green &&
				color.green <= maxColorRaw.green
	}

	/** Checks whether the detected color is in the specified range (inclusive). */
	fun isColorInRange(minColor: Color, maxColor: Color): Boolean {
		val color = colorSensor.color
		return color.red >= minColor.red &&
				color.red <= maxColor.red &&
				color.blue >= minColor.blue &&
				color.blue <= maxColor.blue &&
				color.green >= minColor.green &&
				color.green <= maxColor.green
	}

	/** Checks whether the detected proximity is in the specified range (inclusive). */
	fun isObjectInProximityRange(minProximity: Double, maxProximity: Double): Boolean {
		return proximityRaw >= minProximity && proximityRaw <= maxProximity
	}


	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("HaColorSensor")
		builder.addIntegerProperty("Red", { redRaw.toLong() }, null)
		builder.addIntegerProperty("Green", { greenRaw.toLong() }, null)
		builder.addIntegerProperty("Blue", { blueRaw.toLong() }, null)
		builder.addIntegerProperty("IR", { ir.toLong() }, null)
		builder.addDoubleProperty("Red %", { red }, null)
		builder.addDoubleProperty("Green %", { green }, null)
		builder.addDoubleProperty("Blue %", { blue }, null)
		builder.addDoubleProperty("Proximity (CM)", { proximityCentimeters.centimeters }, null)
		builder.addIntegerProperty("Proximity (0-2047)", { proximityRaw.toLong() }, null)
	}

	companion object {
		private const val MIN_PROXIMITY = 0
		private const val MAX_PROXIMITY = 2047
		private const val MIN_DISTANCE_CM = 1
		private const val MAX_DISTANCE_CM = 10
	}
}
