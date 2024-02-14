package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.PercentOutput
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.RobotMap.Intake as IntakeMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase() {
	private val floorIntakeMotor = HaTalonFX(IntakeMap.FLOOR_INTAKE_MOTOR_ID).apply {
		configurator.apply(TalonFXConfiguration())
		// TODO: Verify positive output intakes
		inverted = false
		idleMode = IdleMode.kBrake
		configurator.apply(Constants.CURRENT_LIMIT_CONFIGURATION)
	}

	private val intakeToLoaderMotor = HaTalonFX(IntakeMap.INTAKE_TO_LOADER_MOTOR_ID).apply {
		configurator.apply(TalonFXConfiguration())
		inverted = false
		idleMode = IdleMode.kBrake
		configurator.apply(Constants.CURRENT_LIMIT_CONFIGURATION)
		// TODO: Check if this motor's direction needs to oppose the other one or not
		setControl(Follower(IntakeMap.FLOOR_INTAKE_MOTOR_ID, false))
	}

	fun set(output: PercentOutput) {
		floorIntakeMotor.set(output)
	}

	val isRunning: Boolean
		get() = abs(floorIntakeMotor.get()) > 0.0

	private var idleMode = IdleMode.kBrake
		set(value) {
			intakeToLoaderMotor.idleMode = value
			floorIntakeMotor.idleMode = value
			field = value
		}
}
