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

		//TODO: Set the channels
		const val LEFT_TRAP_SWITCH = 0
		const val RIGHT_TRAP_SWITCH = 0
	}

	object Intake {
		const val TOP_MOTOR_ID = 21
		const val BOTTOM_MOTOR_ID = 20
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
		const val BEAM_BREAK_CHANNEL = 0
	}

	object Swerve {
		const val CANBUS_NAME = "SwerveBus"
		const val PIGEON_ID = 2

		object FrontLeft {
			const val DRIVE_MOTOR_ID = 15
			const val STEER_MOTOR_ID = 5
			const val CANCODER_ID = 6
		}

		object FrontRight {
			const val DRIVE_MOTOR_ID = 16
			const val STEER_MOTOR_ID = 7
			const val CANCODER_ID = 8
		}

		object BackLeft {
			const val DRIVE_MOTOR_ID = 17
			const val STEER_MOTOR_ID = 9
			const val CANCODER_ID = 10
		}

		object BackRight {
			const val DRIVE_MOTOR_ID = 18
			const val STEER_MOTOR_ID = 11
			const val CANCODER_ID = 12
		}
	}
}