package frc.robot.subsystems.arm

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.hamosad1657.lib.motors.HaTalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Arm as ArmMap

object ArmSubsystem : SubsystemBase() {
    // TODO: Check if limit is wired normally false or normally false.

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

    fun setNeutralMode(neutralMode: NeutralMode) {
        leftMotor.setNeutralMode(neutralMode)
        rightMotor.setNeutralMode(neutralMode)
    }

    fun set(percentOutput: Double) {
        leftMotor.set(percentOutput)
    }

    val isAtForwardLimit get() = forwardLimit.get()

    val isAtReverseLimit: Boolean
        get() {
            if ((leftMotor.statorCurrent >= ArmConstants.STATOR_CURRENT_AT_REVERSE_LIMIT) &&
                reverseLimitTimer.hasElapsed(ArmConstants.AT_REVERSE_LIMIT_TIME)
            ) {
                return true
            } else {
                reverseLimitTimer.restart()
                return false
            }
        }
}