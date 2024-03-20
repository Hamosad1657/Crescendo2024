package frc.robot.subsystems.leds

object LEDsConstants {
	const val LENGTH = 24

	const val ACTION_FINISHED_MODE_TIMEOUT = 1.0

	const val ACTION_FINISHED_MODE_BLINK_TIME = 0.1
	const val ACTION_FAILING_BLINK_TIME = 0.1

	const val WAIT_WITH_NOTE_DELAY = 0.15

	enum class LEDsMode {
		DEFAULT,
		ROBOT_DISABLED,
		ACTION_FINISHED,
		ACTION_FAILING,
		COLLECT,
		SHOOT,
		DYNAMIC_SHOOT,
		TELEOP_SHOOTER
	}
}