package frc.robot.subsystems.arm

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.hamosad1657.lib.motors.HaTalonSRX
import com.hamosad1657.lib.subsystemutils.setNameToClassName
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.arm.ArmConstants.AT_REVERSE_LIMIT_TIME_SEC
import frc.robot.subsystems.arm.ArmConstants.STATOR_CURRENT_AT_REVERSE_LIMIT
import frc.robot.RobotMap.Arm as ArmMap

object ArmSubsystem : SubsystemBase() {
    init {
        setNameToClassName()
    }

    private val leftMotor = HaTalonSRX(ArmMap.MOTOR_ID)
    private val rightMotor = HaTalonSRX(ArmMap.MOTOR_ID)

    init {
        leftMotor.inverted = false // TODO: Verify positive output opens arm.
        rightMotor.inverted = false
        rightMotor.follow(leftMotor)
    }

    private val forwardLimit = DigitalInput(ArmMap.FORWARD_LIMIT_CHANNEL)

    private val reverseLimitTimer = Timer().apply {
        start()
    }

    var neutralMode: NeutralMode = NeutralMode.Brake
        set(value) {
            leftMotor.setNeutralMode(value)
            rightMotor.setNeutralMode(value)
            field = value
        }

    fun set(percentOutput: Double) {
        leftMotor.set(percentOutput)
    }

    // TODO: Check if limit switch is wired normally false or normally true.
    val isAtForwardLimit get() = forwardLimit.get()

    /**
     * True if stator current was over [STATOR_CURRENT_AT_REVERSE_LIMIT]
     * for over [AT_REVERSE_LIMIT_TIME_SEC].
     */
    val isAtReverseLimit: Boolean
        get() {
            if ((leftMotor.statorCurrent >= STATOR_CURRENT_AT_REVERSE_LIMIT) &&
                reverseLimitTimer.hasElapsed(AT_REVERSE_LIMIT_TIME_SEC)
            ) {
                return true
            } else {
                reverseLimitTimer.restart()
                return false
            }
        }
}