package frc.robot.subsystems.loader

import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.units.FractionalOutput
import com.hamosad1657.lib.units.toIdleMode
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.RobotMap.Loader as LoaderMap

object LoaderSubsystem : SubsystemBase() {

	private val motor = CANSparkFlex(LoaderMap.MOTOR_ID, MotorType.kBrushless)
	private val beamBreak = AnalogInput(LoaderMap.BEAM_BREAK_CHANNEL)

	var neutralMode = NeutralModeValue.Brake
		set(value) {
			motor.setIdleMode(value.toIdleMode())
			field = value
		}

	// TODO: Check if beam-break sensor is wired normally-true or normally-false
	/** Beam-break is positioned between loader and shooter. */
	val isNoteDetected: Boolean
		get() = beamBreak.value >= LoaderConstants.ANALOG_INPUT_THRESHOLD

	fun set(output: FractionalOutput) {
		motor.set(output)
	}

	val isRunning: Boolean
		get() = abs(motor.get()) > 0.0

	override fun initSendable(builder: SendableBuilder) {
		super.initSendable(builder)
		builder.addBooleanProperty("Is note detected", { isNoteDetected }, null)
		builder.addBooleanProperty("Running", { isRunning }, null)
	}
}