package frc.robot.subsystems.loader

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.hamosad1657.lib.motors.HaCANSparkMax
import com.hamosad1657.lib.subsystemutils.setNameToClassName
import com.hamosad1657.lib.units.toIdleMode
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Loader as LoaderMap

object LoaderSubsystem : SubsystemBase() {
	init {
		setNameToClassName()
	}

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

	fun setLoader(percentOutput: Double) {
		motor.set(percentOutput)
	}

	val running: Boolean
		get() = motor.get() > 0.0
}