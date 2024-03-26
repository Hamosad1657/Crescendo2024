package com.hamosad1657.lib.leds

import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.Timer

/** ONLY CREATE ONE INSTANCE OF THIS CLASS. */
class LEDStrip(private val length: Int, pwmPort: Int) {
	// --- LEDs ---

	private val ledBuffer = AddressableLEDBuffer(length)
	private val ledStrip = AddressableLED(pwmPort).apply {
		setLength(length)
		setData(ledBuffer)
		start()
	}


	// --- LEDs State ---

	private val blinkTimer = Timer()

	var currentColor = LEDS_OFF


	// --- State Getters ---

	val areLEDsOn: Boolean
		get() =
			(ledBuffer.getRed(0) != 0) ||
				(ledBuffer.getGreen(0) != 0) ||
				(ledBuffer.getBlue(0) != 0)


	// --- LEDs Control ---

	fun setColor(color: RGBColor) {
		for (i in 0..<length) {
			ledBuffer.setRGB(i, color.red, color.green, color.blue)
		}
		ledStrip.setData(ledBuffer)

		if (color != LEDS_OFF) currentColor = color
	}

	fun setColorAlternating(color: RGBColor) {
		for (i in 0..<length step 5) {
			ledBuffer.setRGB(i, color.red, color.green, color.blue)
		}
		ledStrip.setData(ledBuffer)

		if (color != LEDS_OFF) currentColor = color
	}

	fun toggleLEDs() {
		setColor(
			if (areLEDsOn) LEDS_OFF
			else currentColor
		)
	}

	/** Should be called periodically. */
	fun blink(blinkTime: Seconds) {
		blinkTimer.start()
		if (blinkTimer.hasElapsed(blinkTime)) {
			toggleLEDs()
			blinkTimer.restart()
		}
	}

	companion object {
		val LEDS_OFF = RGBColor.BLACK
	}
}