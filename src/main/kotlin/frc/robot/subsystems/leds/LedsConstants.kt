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
		COLLECT, SHOOT, BLINK_READY, DEFAULT,
	}

	const val LENGTH = 0
	const val READY_BLINK_TIME = 0.2
}