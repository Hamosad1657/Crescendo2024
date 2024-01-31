package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.subsystemutils.setNameToClassName
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.RobotMap.Intake as IntakeMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase() {

	private val intakeToLoaderMotor = HaTalonFX(IntakeMap.INTAKE_TO_LOADER_MOTOR_ID).apply {
		isSafetyEnabled = true
		// TODO: Verify positive output intakes
		inverted = false
		configurator.apply(Constants.CURRENT_LIMIT_CONFIGURATION)
	}

	private val floorIntakeMotor = HaTalonFX(IntakeMap.GROUND_INTAKE_MOTOR_ID).apply {
		isSafetyEnabled = true
		configurator.apply(Constants.CURRENT_LIMIT_CONFIGURATION)
		// TODO: Check if this motor's direction needs to oppose the other one or not
		this.setControl(Follower(IntakeMap.INTAKE_TO_LOADER_MOTOR_ID, false))
	}

	// TODO: Change default neutral mode
	var neutralMode = NeutralModeValue.Coast
		set(value) {
			intakeToLoaderMotor.setNeutralMode(value)
			field = value
		}

	fun set(percentOutput: Double) {
		intakeToLoaderMotor.set(percentOutput)
	}

	val isRunning: Boolean
		get() = abs(floorIntakeMotor.get()) > 0.0
}