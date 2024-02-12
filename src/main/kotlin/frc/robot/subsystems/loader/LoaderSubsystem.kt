package frc.robot.subsystems.loader

import com.hamosad1657.lib.commands.andThen
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.PercentOutput
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.*
import frc.robot.Robot
import kotlin.math.abs
import frc.robot.RobotMap.Loader as LoaderMap

object LoaderSubsystem : SubsystemBase() {
	private val motor = HaSparkFlex(LoaderMap.MOTOR_ID, MotorType.kBrushless).apply {
		// TODO: Verify positive output loads
		inverted = false
		idleMode = IdleMode.kBrake
	}
	private val beamBreak = DigitalInput(LoaderMap.BEAM_BREAK_CHANNEL)

	// TODO: Check if beam-break sensor is wired normally-true or normally-false
	/** Beam-break is positioned between loader and shooter. */
	val isNoteDetected get() = beamBreak.get()

	fun set(output: PercentOutput) {
		motor.set(output)
	}

	val isRunning: Boolean
		get() = abs(motor.get()) > 0.0

	private val disabledCoastCommand =
		WaitUntilCommand(Robot.DISABLED_COAST_DELAY_SECONDS) andThen
			InstantCommand({ motor.idleMode = IdleMode.kCoast })

	// TODO: Check if works as expected.
	fun disabledInit() {
		CommandScheduler.getInstance().schedule(disabledCoastCommand)
	}

	fun disabledExit() {
		CommandScheduler.getInstance().cancel(disabledCoastCommand)
		motor.idleMode = IdleMode.kBrake
	}

	override fun initSendable(builder: SendableBuilder) {
		super.initSendable(builder)
		builder.addBooleanProperty("Is note detected", { isNoteDetected }, null)
		builder.addBooleanProperty("Running", { isRunning }, null)
	}
}