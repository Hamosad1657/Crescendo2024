package frc.robot

/**
 * This objects contains DIO channel numbers, CAN IDs, USB port numbers and the like.
 */
object RobotMap {
	const val DRIVER_A_CONTROLLER_PORT = 0
	const val DRIVER_B_CONTROLLER_PORT = 1
	const val TESTING_CONTROLLER_PORT = 5

	object Climbing {
		const val LEFT_FRONT_MOTOR_ID = 25
		const val LEFT_BACK_MOTOR_ID = 26
		const val RIGHT_FRONT_MOTOR_ID = 27
		const val RIGHT_BACK_MOTOR_ID = 28
		const val LEFT_CLOSED_LIMIT_CHANNEL = 3
		const val LEFT_OPENED_LIMIT_CHANNEL = 2
		const val RIGHT_CLOSED_LIMIT_CHANNEL = 4
		const val RIGHT_OPENED_LIMIT_CHANNEL = 5
	}

	object Intake {
		const val INTAKE_TO_LOADER_MOTOR_ID = 21
		const val FLOOR_INTAKE_MOTOR_ID = 20
	}

	object Shooter {
		const val UPPER_MOTOR_ID = 23
		const val LOWER_MOTOR_ID = 24

		object Angle {
			const val MOTOR_ID = 4
			const val CANCODER_ID = 3

			const val MIN_ANGLE_LIMIT_CHANNEL = 0
			const val MAX_ANGLE_LIMIT_CHANNEL = 1
		}
	}

	object Arm {
		const val MOTOR_ID = 0
		const val LEFT_FORWARD_LIMIT_CHANNEL = 0
		const val LEFT_REVERSE_LIMIT_CHANNEL = 0
		const val RIGHT_FORWARD_LIMIT_CHANNEL = 0
		const val RIGHT_REVERSE_LIMIT_CHANNEL = 0
	}

	object Loader {
		const val MOTOR_ID = 22
		const val BEAM_BREAK_CHANNEL = 6
	}
}