package frc.robot.subsystems.loader

import com.ctre.phoenix6.controls.VoltageOut
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.Volts
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.RobotMap.Loader as LoaderMap
import frc.robot.subsystems.loader.LoaderConstants as Constants

object LoaderSubsystem : SubsystemBase() {
	// --- Motors & Sensors ---

	private val motor = HaTalonFX(LoaderMap.MOTOR_ID).apply {
		restoreFactoryDefaults()
		inverted = false
		idleMode = IdleMode.kBrake
		configurator.apply(Constants.MOTORS_CURRENT_LIMIT)
	}

	private val beamBreak = DigitalInput(LoaderMap.BEAM_BREAK_CHANNEL)


	// --- Motors Configuration ---

	var idleMode = IdleMode.kBrake
		set(value) {
			motor.idleMode = value
			field = value
		}


	// --- State Getters ---

	/** Beam-break is positioned between loader and shooter. */
	val isNoteDetected: Boolean get() = !beamBreak.get()

	val isRunning: Boolean get() = abs(motor.get()) > 0.0


	// --- Motors Control ---

	private val controlRequestVoltage = VoltageOut(0.0).apply { EnableFOC = false }

	fun setVoltage(voltage: Volts) {
		motor.setControl(controlRequestVoltage.apply { Output = voltage })
	}

	fun stopMotors() {
		motor.stopMotor()
	}


	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("Subsystem")
		builder.addStringProperty("Command", { currentCommand?.name ?: "none" }, null)
		builder.addBooleanProperty("Note detected", { isNoteDetected }, null)
		builder.addBooleanProperty("Running", { isRunning }, null)
	}
}