package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase() {
	private val motor = WPI_TalonSRX(RobotMap.Intake.MOTOR_ID)

	var idleMode: IdleMode = IdleMode.kCoast
		set(value) {
			when (value) {
				IdleMode.kBrake -> motor.setNeutralMode(NeutralMode.Brake)
				IdleMode.kCoast -> motor.setNeutralMode(NeutralMode.Coast)
			}
			field = value
		}

	init {
		motor.inverted = false // TODO: Verify positive output intakes
		motor.setNeutralMode(NeutralMode.Coast) // TODO: Decide intake idle mode
		motor.configSupplyCurrentLimit(Constants.supplyCurrentLimitConfiguration)
		motor.isSafetyEnabled = true
		idleMode = IdleMode.kCoast
	}

	fun set(percentOutput: Double) {
		motor.set(percentOutput)
	}
}