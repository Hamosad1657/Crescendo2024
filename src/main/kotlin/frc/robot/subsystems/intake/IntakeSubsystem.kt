package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.hamosad1657.lib.motors.HaTalonSRX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Intake as IntakeMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase() {
    private val motor = HaTalonSRX(IntakeMap.MOTOR_ID).apply {
        isSafetyEnabled = true
        // TODO: Verify positive output intakes
        inverted = false
        configSupplyCurrentLimit(Constants.SUPPLY_CURRENT_LIMIT)
    }

    // TODO: Change default neutral mode
    var neutralMode = NeutralMode.Coast
        set(value) {
            motor.setNeutralMode(value)
            field = value
        }
    
    fun set(percentOutput: Double) {
        motor.set(percentOutput)
    }
}