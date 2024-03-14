package frc.robot.subsystems.leds

object LedsConstants {
	data class RGBColor(val red: Int, val green: Int, val blue: Int) {
		companion object {
			val TEAM_GREEN = RGBColor(22, 87, 0)
			val PURE_GREEN = RGBColor(0, 255, 0)
			val WHITE = RGBColor(255, 255, 255)
			val RED = RGBColor(255, 0, 0)
			val BLUE = RGBColor(0, 0, 255)
			val YELLOW = RGBColor(255, 255, 0)
			val CYAN = RGBColor(0, 255, 255)
			val MAGENTA = RGBColor(255, 0, 255)
			val DARK = RGBColor(0, 0, 0)
		}
	}

	enum class LEDsMode {
		COLLECT, SHOOT, BLINK_DONE, DEFAULT,
	}

	const val LENGTH = 24
	const val BLINK_DONE_TIME = 0.2
	const val EXIT_BLINK_DONE_MODE_TIMEOUT = 2.0
}