package frc.robot.subsystems.leds

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.RobotMap
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LEDsConstants.ACTION_FINISHED_MODE_TIMEOUT
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.*
import frc.robot.subsystems.leds.LEDsConstants.RGBColor
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.subsystems.leds.LEDsConstants as Constants

object LEDsSubsystem : SubsystemBase() {
	// --- LEDs ---

	private val blinkTimer = Timer()
	private val actionFinishedModeExitTimer = Timer()
	private val collectWithNoteTimer = Timer()

	private val ledsBuffer = AddressableLEDBuffer(Constants.LENGTH)
	private val ledStrip = AddressableLED(RobotMap.Leds.PWM_PORT).apply {
		setLength(Constants.LENGTH)
		setData(ledsBuffer)
		start()
	}


	// --- LEDs State ---

	var currentMode: LEDsMode = ROBOT_DISABLED

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
	private fun blink(blinkTime: Seconds) {
		blinkTimer.start()
		if (blinkTimer.hasElapsed(blinkTime)) {
			toggleLeds()
			blinkTimer.restart()
		}
	}


	// --- Modes Periodic Functions ---

	private fun actionFinishedMode() {
		blink(Constants.ACTION_FINISHED_MODE_BLINK_TIME)
	}

	private fun collectMode() {
		setColor(
			if (
				IntakeSubsystem.isCollectingNote &&
				collectWithNoteTimer.hasElapsed(Constants.WAIT_WITH_NOTE_DELAY)
			)
				RGBColor.MAGENTA
			else RGBColor.YELLOW
		)
	}

	private fun shootMode() {
		setColor(
			if (ShooterSubsystem.isWithinTolerance) RGBColor.MAGENTA
			else RGBColor.YELLOW
		)
	}

	private fun dynamicShootMode() {
		if (!SwerveSubsystem.isInVisionRange) {
			currentColor = RGBColor.ORANGE
			blink(0.1)
		} else shootMode()
	}

	private fun defaultMode() {
		setColor(RGBColor.LEDS_OFF)
	}

	private fun robotDisabledMode() {
		setColor(
			if (Robot.alliance == Alliance.Blue) RGBColor.BLUE
			else RGBColor.RED
		)
	}


	// --- LEDs Commands Control ---

	fun actionFinished(interrupted: Boolean) {
		actionFinishedModeExitTimer.restart()
		currentMode =
			if (interrupted) {
				DEFAULT
			} else {
				setColor(RGBColor.GREEN)
				ACTION_FINISHED
			}
	}

	fun setToDefaultMode() {
		if (currentMode != ACTION_FINISHED) currentMode = DEFAULT
	}

	fun setModeCommand(mode: LEDsMode): Command =
		instantCommand {
			currentMode = mode
			if (mode == COLLECT) collectWithNoteTimer.restart()
		}


	// --- Periodic ---

	override fun periodic() {
		when (currentMode) {
			ACTION_FINISHED -> {
				actionFinishedMode()
				if (actionFinishedModeExitTimer.hasElapsed(ACTION_FINISHED_MODE_TIMEOUT)) {
					currentMode = DEFAULT
					actionFinishedModeExitTimer.stop()
				}
			}

			COLLECT -> collectMode()
			SHOOT -> shootMode()
			DYNAMIC_SHOOT -> dynamicShootMode()
			DEFAULT -> defaultMode()
			ROBOT_DISABLED -> robotDisabledMode()
		}
	}
}