package com.hamosad1657.lib.led

import com.hamosad1657.lib.commands.instantCommand
import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command

class LEDStrip(private val length: Int, pwmPort: Int) {
	// --- LEDs ---

	private val ledBuffer = AddressableLEDBuffer(length)
	private val ledStrip = AddressableLED(pwmPort).apply {
		setLength(length)
		setData(ledBuffer)
		start()
	}


	// --- LEDs State ---
	
	private var currentColor = LEDS_OFF

	private val blinkTimer = Timer()
	private val actionFinishedModeExitTimer = Timer()


	// --- State Getters ---

	val areLedsOn: Boolean
		get() =
			(ledBuffer.getRed(0) != 0) ||
				(ledBuffer.getGreen(0) != 0) ||
				(ledBuffer.getBlue(0) != 0)


	// --- LEDs Control ---

	private fun setColor(color: RGBColor) {
		for (i in 0..<length) {
			ledBuffer.setRGB(i, color.red, color.green, color.blue)
		}
		ledStrip.setData(ledBuffer)

		if (color != LEDS_OFF) currentColor = color
	}

	private fun toggleLEDs() {
		setColor(
			if (areLedsOn) LEDS_OFF
			else currentColor
		)
	}

	/** Should be called periodically. */
	private fun blink(blinkTime: Seconds) {
		blinkTimer.start()
		if (blinkTimer.hasElapsed(blinkTime)) {
			toggleLEDs()
			blinkTimer.restart()
		}
	}


	// --- Modes Periodic Functions ---

	fun defaultMode() {
		setColor(LEDS_OFF)
	}

	fun robotDisabledMode(alliance: Alliance) {
		setColor(
			if (alliance == Alliance.Blue) RGBColor.BLUE
			else RGBColor.RED
		)
	}


	// --- LEDs Commands Control ---

	fun actionFinished(interrupted: Boolean) {
		actionFinishedModeExitTimer.restart()
		currentMode =
			if (interrupted) {
				DEFAULT
			} else {
				setColor(RGBColor.PURE_GREEN)
				ACTION_FINISHED
			}
	}

	fun setToDefaultMode() {
		if (currentMode != ACTION_FINISHED) currentMode = DEFAULT
	}

	fun setModeCommand(mode: LEDsMode): Command =
		instantCommand {
			currentMode = mode
			if (mode == COLLECT) {
				collectWithNoteTimer.restart()
				hasCollectedNote = false
			}
		}

	/**
	 * This should be called in a Subsystem.periodic method or any similar method.
	 *
	 * Sets the LEDs' color based on the current mode. Calls the
	 */
	fun update() {

	}

	companion object {
		val LEDS_OFF = RGBColor.BLACK
	}
}