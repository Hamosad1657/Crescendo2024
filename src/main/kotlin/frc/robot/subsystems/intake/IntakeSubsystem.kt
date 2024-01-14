package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase() {
	private val motor = WPI_TalonSRX(RobotMap.Intake.MOTOR_ID)

	// TODO: Change default neutral mode
	var neutralMode: NeutralMode = NeutralMode.Coast
		set(value) {
			motor.setNeutralMode(value)
			field = value
		}

	init {
		motor.inverted = false // TODO: Verify positive output intakes
		motor.configSupplyCurrentLimit(Constants.SUPPLY_CURRENT_LIMIT)
		motor.isSafetyEnabled = true
	}

	fun set(percentOutput: Double) {
		motor.set(percentOutput)
	}
}