package frc.robot.subsystems.arm

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.hamosad1657.lib.motors.HaCANSparkMax
import com.hamosad1657.lib.subsystemutils.setNameToClassName
import com.hamosad1657.lib.units.toIdleMode
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.arm.ArmConstants.CURRENT_LIMIT_AMP
import frc.robot.RobotMap.Arm as ArmMap

object ArmSubsystem : SubsystemBase() {
    init {
        setNameToClassName()
    }

    private val leftMotor = HaCANSparkMax(ArmMap.MOTOR_ID)
    private val rightMotor = HaCANSparkMax(ArmMap.MOTOR_ID)

    init {
        leftMotor.inverted = false // TODO: Verify positive output opens arm.
        rightMotor.inverted = false

        leftMotor.setSmartCurrentLimit(CURRENT_LIMIT_AMP)
        rightMotor.setSmartCurrentLimit(CURRENT_LIMIT_AMP)
        
        rightMotor.follow(leftMotor)
    }

    private val forwardLimit = DigitalInput(ArmMap.FORWARD_LIMIT_CHANNEL)
    private val reverseLimit = DigitalInput(ArmMap.REVERSE_LIMIT_CHANNEL)


    var neutralMode: NeutralMode = NeutralMode.Brake
        set(value) {
            leftMotor.setIdleMode(value.toIdleMode())
            rightMotor.setIdleMode(value.toIdleMode())
            field = value
        }

    fun set(percentOutput: Double) {
        leftMotor.set(percentOutput)
    }

    // TODO: Check if limit switch is wired normally false or normally true.
    val isAtForwardLimit get() = forwardLimit.get()
    val isAtReverseLimit get() = reverseLimit.get()
}