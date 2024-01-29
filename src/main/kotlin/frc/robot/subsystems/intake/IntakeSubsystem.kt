package frc.robot.subsystems.intake

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.hamosad1657.lib.motors.HaTalonSRX
import com.hamosad1657.lib.subsystemutils.setNameToClassName
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Intake as IntakeMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase() {
	init {
		setNameToClassName()
	}

	private val intakeToLoaderMotor = HaTalonSRX(IntakeMap.MOTOR_1_ID).apply {
		isSafetyEnabled = true
		// TODO: Verify positive output intakes
		inverted = false
		configSupplyCurrentLimit(Constants.SUPPLY_CURRENT_LIMIT)
	}

	private val floorIntakeMotor = HaTalonSRX(IntakeMap.MOTOR_2_ID).apply {
		isSafetyEnabled = true
		// TODO: Verify positive output intakes
		inverted = false
		configSupplyCurrentLimit(Constants.SUPPLY_CURRENT_LIMIT)
		follow(intakeToLoaderMotor)
	}

	// TODO: Change default neutral mode
	var neutralMode = NeutralMode.Coast
		set(value) {
			intakeToLoaderMotor.setNeutralMode(value)
			field = value
		}

	fun set(percentOutput: Double) {
		intakeToLoaderMotor.set(percentOutput)
	}
}