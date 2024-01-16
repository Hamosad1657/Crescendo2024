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

    object Grabber {

    }

    object Intake {

    }

    object Shooter {

    }

    object Swerve {

    }

    object Arm {
        const val MOTOR_ID = 0
        const val FORWARD_LIMIT_CHANNEL = 0
        const val REVERSE_LIMIT_CHANNEL = 0
    }
}