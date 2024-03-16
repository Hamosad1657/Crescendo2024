package frc.robot.subsystems.leds

object LEDsConstants {
	data class RGBColor(val red: Int, val green: Int, val blue: Int) {
		companion object {
			val GREEN = RGBColor(0, 255, 0)
			val WHITE = RGBColor(255, 255, 255)
			val RED = RGBColor(255, 0, 0)
			val BLUE = RGBColor(0, 0, 255)
			val YELLOW = RGBColor(255, 255, 0)
			val ORANGE = RGBColor(255, 30, 0)
			val CYAN = RGBColor(0, 255, 255)
			val MAGENTA = RGBColor(255, 0, 255)
			val LEDS_OFF = RGBColor(0, 0, 0)
		}
	}

	enum class LEDsMode {
		ACTION_FINISHED,
		COLLECT,
		SHOOT,
		DYNAMIC_SHOOT,
		DEFAULT,
		ROBOT_DISABLED,
	}

	const val LENGTH = 24

	const val ACTION_FINISHED_MODE_BLINK_TIME = 0.15
	const val ACTION_FINISHED_MODE_TIMEOUT = 6.5 * ACTION_FINISHED_MODE_BLINK_TIME

	const val WAIT_WITH_NOTE_DELAY = 0.15
}