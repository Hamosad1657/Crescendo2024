package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.hamosad1657.lib.motors.HaTalonSRX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Intake as IntakeMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase() {
    private val mainMotor = HaTalonSRX(IntakeMap.MOTOR_1_ID).apply {
        isSafetyEnabled = true
        // TODO: Verify positive output intakes
        inverted = false
        configSupplyCurrentLimit(Constants.SUPPLY_CURRENT_LIMIT)
    }

    private val followerMotor = HaTalonSRX(IntakeMap.MOTOR_2_ID).apply {
        isSafetyEnabled = true
        // TODO: Verify positive output intakes
        inverted = false
        configSupplyCurrentLimit(Constants.SUPPLY_CURRENT_LIMIT)
        follow(mainMotor)
    }

    // TODO: Change default neutral mode
    var neutralMode = NeutralMode.Coast
        set(value) {
            mainMotor.setNeutralMode(value)
            field = value
        }

    fun set(percentOutput: Double) {
        mainMotor.set(percentOutput)
    }
}