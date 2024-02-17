package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.Volts
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.RobotMap.Intake as IntakeMap

object IntakeSubsystem : SubsystemBase() {
	private val bottomMotor = HaTalonFX(IntakeMap.BOTTOM_MOTOR_ID).apply {
		configurator.apply(TalonFXConfiguration())
		inverted = false
		idleMode = IdleMode.kBrake
	}

	private val topMotor = HaTalonFX(IntakeMap.TOP_MOTOR_ID).apply {
		configurator.apply(TalonFXConfiguration())
		inverted = false
		idleMode = IdleMode.kBrake
	}

	private val controlRequestBottomVoltage = VoltageOut(0.0).apply { EnableFOC = false }
	private val controlRequestTopVoltage = VoltageOut(0.0).apply { EnableFOC = false }

	fun setVoltage(bottomVoltage: Volts, topVoltage: Volts) {
		bottomMotor.setControl(controlRequestBottomVoltage.apply { Output = bottomVoltage })
		topMotor.setControl(controlRequestTopVoltage.apply { Output = topVoltage })
	}

	fun stop() {
		bottomMotor.stopMotor()
		topMotor.stopMotor()
	}

	val isRunning: Boolean
		get() = abs(bottomMotor.get()) > 0.0

	private var idleMode = IdleMode.kBrake
		set(value) {
			topMotor.idleMode = value
			bottomMotor.idleMode = value
			field = value
		}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("Subsystem")
		builder.addStringProperty("Command", { currentCommand?.name ?: "none" }, null)

		builder.addBooleanProperty("Is running", { isRunning }, null)
		builder.addDoubleProperty("Bottom motor output", { bottomMotor.get() }, null)
		builder.addDoubleProperty("Top motor output", { topMotor.get() }, null)
	}
}
