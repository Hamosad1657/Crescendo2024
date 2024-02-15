package frc.robot.subsystems.loader

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.Volts
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.loader.LoaderConstants.ANALOG_READ_NOTE_DETECTED_THRESHOLD
import kotlin.math.abs
import frc.robot.RobotMap.Loader as LoaderMap

object LoaderSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(LoaderMap.MOTOR_ID).apply {
		configurator.apply {
			TalonFXConfiguration()
		}
		// TODO: Verify positive output loads
		inverted = false
		idleMode = IdleMode.kBrake
	}

	private val beamBreak = AnalogInput(LoaderMap.BEAM_BREAK_CHANNEL)

	// TODO: Check if beam-break sensor is wired normally-true or normally-false
	/** Beam-break is positioned between loader and shooter. */
	val isNoteDetected: Boolean get() = beamBreak.value <= ANALOG_READ_NOTE_DETECTED_THRESHOLD

	private val controlRequestVoltage = VoltageOut(0.0).apply { EnableFOC = false }
	
	fun setVoltage(voltage: Volts) {
		motor.setControl(controlRequestVoltage.apply { Output = voltage })
	}

	fun stop() {
		motor.stopMotor()
	}

	val isRunning: Boolean
		get() = abs(motor.get()) > 0.0

	override fun initSendable(builder: SendableBuilder) {
		super.initSendable(builder)
		builder.addDoubleProperty("Analog channel 0 voltage", { beamBreak.value.toDouble() }, null)
		builder.addBooleanProperty("Is note detected", { isNoteDetected }, null)
		builder.addBooleanProperty("Running", { isRunning }, null)
	}
}