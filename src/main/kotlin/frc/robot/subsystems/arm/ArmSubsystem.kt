package frc.robot.subsystems.arm

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.hamosad1657.lib.motors.HaTalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Arm as ArmMap

object ArmSubsystem : SubsystemBase() {
    private val motor = HaTalonSRX(ArmMap.MOTOR_ID)
    private val forwardLimit = DigitalInput(ArmMap.FORWARD_LIMIT_CHANNEL)
    private val reverseLimit = DigitalInput(ArmMap.REVERSE_LIMIT_CHANNEL)

    fun setNeutralMode(neutralMode: NeutralMode) {
        motor.setNeutralMode(neutralMode)
    }

    fun set(percentOutput: Double) {
        motor.set(percentOutput)
    }
}