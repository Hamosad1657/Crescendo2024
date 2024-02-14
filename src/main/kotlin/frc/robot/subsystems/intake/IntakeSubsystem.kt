package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.PercentOutput
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.RobotMap.Intake as IntakeMap

object IntakeSubsystem : SubsystemBase() {
	private val bottomMotor = HaTalonFX(IntakeMap.BOTTOM_MOTOR_ID).apply {
		configurator.apply(TalonFXConfiguration())
		// TODO: Verify positive output intakes
		inverted = false
		idleMode = IdleMode.kBrake
	}

	private val topMotor = HaTalonFX(IntakeMap.TOP_MOTOR_ID).apply {
		configurator.apply(TalonFXConfiguration())
		inverted = false
		idleMode = IdleMode.kBrake
	}

	fun set(bottomOutput: PercentOutput, topOutput: PercentOutput) {
		bottomMotor.set(bottomOutput)
		topMotor.set(topOutput)
	}

	fun stop() {
		bottomMotor.set(0.0)
		topMotor.set(0.0)
	}

	val isRunning: Boolean
		get() = abs(bottomMotor.get()) > 0.0

	private var idleMode = IdleMode.kBrake
		set(value) {
			topMotor.idleMode = value
			bottomMotor.idleMode = value
			field = value
		}
}
