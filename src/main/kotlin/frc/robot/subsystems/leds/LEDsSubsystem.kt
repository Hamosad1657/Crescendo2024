package frc.robot.subsystems.leds

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LedsConstants.ACTION_FINISHED_MODE_TIMEOUT
import frc.robot.subsystems.leds.LedsConstants.LEDsMode
import frc.robot.subsystems.leds.LedsConstants.LEDsMode.*
import frc.robot.subsystems.leds.LedsConstants.RGBColor
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.leds.LedsConstants as Constants

object LEDsSubsystem : SubsystemBase() {
	// --- LEDs ---

	private val blinkTimer = Timer()
	private val exitActionFinishedModeTimer = Timer()

	private val ledsBuffer = AddressableLEDBuffer(Constants.LENGTH)
	private val ledStrip = AddressableLED(RobotMap.Leds.PWM_PORT).apply {
		setLength(Constants.LENGTH)
		setData(ledsBuffer)
		start()
	}


	// --- LEDs State ---

	var currentMode: LEDsMode = DEFAULT
		private set

	private var currentColor = RGBColor.LEDS_OFF


	// --- State Getters ---

	val areLedsOn: Boolean
		get() =
			(ledsBuffer.getRed(0) != 0) ||
				(ledsBuffer.getGreen(0) != 0) ||
				(ledsBuffer.getBlue(0) != 0)


	// --- LEDs Control ---

	private fun setColor(color: RGBColor) {
		for (i in 0..<Constants.LENGTH) {
			ledsBuffer.setRGB(i, color.red, color.green, color.blue)
		}
		ledStrip.setData(ledsBuffer)

		if (color != RGBColor.LEDS_OFF) currentColor = color
	}

	private fun toggleLeds() {
		setColor(
			if (areLedsOn) RGBColor.LEDS_OFF
			else currentColor
		)
	}

	/** Should be called periodically. */
	private fun blink(color: RGBColor, blinkTime: Seconds) {
		setColor(color)
		blinkTimer.start()
		if (blinkTimer.hasElapsed(blinkTime)) {
			toggleLeds()
			blinkTimer.restart()
		}
	}


	// --- Modes Periodic Functions ---

	private fun actionFinishedMode() {
		blink(RGBColor.PURE_GREEN, Constants.ACTION_FINISHED_MODE_BLINK_TIME)
	}

	private fun collectMode() {
		setColor(
			if (IntakeSubsystem.isBottomMotorUnderLoad) RGBColor.MAGENTA
			else RGBColor.WHITE
		)
	}

	private fun shootMode() {
		setColor(
			if (ShooterSubsystem.isWithinTolerance) RGBColor.MAGENTA
			else RGBColor.WHITE
		)
	}

	private fun defaultMode() {
		setColor(
			if (LoaderSubsystem.isNoteDetected) RGBColor.TEAM_GREEN
			else RGBColor.LEDS_OFF
		)
	}

	private fun robotDisabledMode() {
		when (Robot.alliance) {
			Blue -> RGBColor.BLUE
			Red -> RGBColor.RED
		}
	}


	// --- LEDs Commands Control ---

	fun actionFinished(interrupted: Boolean) {
		currentMode =
			if (interrupted) DEFAULT
			else ACTION_FINISHED
	}

	fun setModeCommand(mode: LEDsMode): Command =
		instantCommand {
			currentMode = mode
		}


	// --- Periodic ---

	override fun periodic() {
		when (currentMode) {
			ACTION_FINISHED -> {
				exitActionFinishedModeTimer.start()
				actionFinishedMode()
				if (exitActionFinishedModeTimer.hasElapsed(ACTION_FINISHED_MODE_TIMEOUT)) {
					currentMode = DEFAULT
					exitActionFinishedModeTimer.stop()
				}
			}

			COLLECT -> collectMode()
			SHOOT -> shootMode()
			DEFAULT -> defaultMode()
			ROBOT_DISABLED -> robotDisabledMode()
		}
	}
}