package frc.robot.subsystems.loader

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.hamosad1657.lib.motors.HaCANSparkMax
import com.hamosad1657.lib.units.toIdleMode
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.abs
import frc.robot.RobotMap.Loader as LoaderMap

object LoaderSubsystem : SubsystemBase() {

	private val motor = HaCANSparkMax(LoaderMap.MOTOR_ID)
	private val beamBreak = DigitalInput(LoaderMap.BEAM_BREAK_CHANNEL)

	var neutralMode = NeutralMode.Brake
		set(value) {
			motor.setIdleMode(value.toIdleMode())
			field = value
		}

	// TODO: Check if beam-break sensor is wired normally-true or normally-false
	/** Beam-break is positioned between loader and shooter. */
	val isNoteDetected get() = beamBreak.get()

	fun set(percentOutput: Double) {
		motor.set(percentOutput)
	}

	val isRunning: Boolean
		get() = abs(motor.get()) > 0.0

	override fun initSendable(builder: SendableBuilder) {
		super.initSendable(builder)
		builder.addBooleanProperty("Is note detected", { isNoteDetected }, null)
		builder.addBooleanProperty("Running", { isRunning }, null)
	}
}