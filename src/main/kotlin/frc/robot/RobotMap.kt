package frc.robot

/**
 * This objects contains DIO channel numbers, CAN IDs, USB port numbers and the like.
 */
object RobotMap {
	const val DRIVER_A_CONTROLLER_PORT = 0
	const val DRIVER_B_CONTROLLER_PORT = 1

	object Climbing {
		const val RIGHT_MAIN_MOTOR_ID = 0
		const val RIGHT_SECONDARY_MOTOR_ID = 0
		const val LEFT_MAIN_MOTOR_ID = 0
		const val LEFT_SECONDARY_MOTOR_ID = 0
	}

	object Intake {
		const val INTAKE_TO_LOADER_MOTOR_ID = 0
		const val GROUND_INTAKE_MOTOR_ID = 0
	}

	object Shooter {
		const val MAIN_MOTOR_ID = 0
		const val SECONDARY_MOTOR_ID = 0

		object Angle {
			const val MOTOR_ID = 0
			const val CANCODER_ID = 0
		}
	}

	object Loader {
		const val MOTOR_ID = 0
		const val BEAM_BREAK_CHANNEL = 0
	}

	object Arm {
		const val MOTOR_ID = 0
		const val LEFT_FORWARD_LIMIT_CHANNEL = 0
		const val LEFT_REVERSE_LIMIT_CHANNEL = 0
		const val RIGHT_FORWARD_LIMIT_CHANNEL = 0
		const val RIGHT_REVERSE_LIMIT_CHANNEL = 0
	}
}