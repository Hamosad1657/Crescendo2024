package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.VoltageOut
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.rps
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.RobotMap.Intake as IntakeMap
import frc.robot.subsystems.intake.IntakeConstants as Constants

object IntakeSubsystem : SubsystemBase() {
	// --- Motors ---

	private val bottomMotor =
		HaTalonFX(IntakeMap.BOTTOM_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configMotor(inverted = false)
		}


	private val topMotor =
		HaTalonFX(IntakeMap.TOP_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configMotor(inverted = false)
		}


	// --- Motors Configuration ---

	private fun HaTalonFX.configMotor(inverted: Boolean) {
		this.inverted = inverted
		idleMode = IdleMode.kBrake
		configurator.apply(Constants.CURRENT_LIMITS_CONFIGS)
	}

	var idleMode = IdleMode.kBrake
		set(value) {
			topMotor.idleMode = value
			bottomMotor.idleMode = value
			field = value
		}


	// --- State Getters ---

	val isRunning: Boolean get() = abs(bottomMotor.get()) > 0.0

	val bottomMotorSpeed: AngularVelocity get() = bottomMotor.velocity.value.rps
	val isCollectingNote: Boolean
		get() = bottomMotorSpeed < Constants.BOTTOM_MOTOR_UNDER_LOAD_THRESHOLD


	// --- Motors Control ---

	private val controlRequestBottomVoltage = VoltageOut(0.0).apply { EnableFOC = false }
	private val controlRequestTopVoltage = VoltageOut(0.0).apply { EnableFOC = false }

	fun setVoltage(bottomVoltage: Volts, topVoltage: Volts) {
		bottomMotor.setControl(controlRequestBottomVoltage.apply { Output = bottomVoltage })
		topMotor.setControl(controlRequestTopVoltage.apply { Output = topVoltage })
	}

	fun stopMotors() {
		bottomMotor.stopMotor()
		topMotor.stopMotor()
	}


	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("Subsystem")
		builder.addStringProperty("Command", { currentCommand?.name ?: "none" }, null)
		builder.addBooleanProperty("Is running", { isRunning }, null)
		builder.addBooleanProperty("Is bottom motor under load", { isCollectingNote }, null)
	}
}
