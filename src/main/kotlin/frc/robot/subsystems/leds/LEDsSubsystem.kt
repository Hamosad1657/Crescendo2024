package frc.robot.subsystems.leds

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.leds.LEDStrip
import com.hamosad1657.lib.leds.RGBColor
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Robot
import frc.robot.subsystems.leds.LEDsConstants.ACTION_FINISHED_MODE_TIMEOUT
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode
import frc.robot.subsystems.leds.LEDsConstants.LEDsMode.*
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.swerve.SwerveSubsystem
import frc.robot.RobotMap.Leds as LedsMap
import frc.robot.subsystems.intake.IntakeSubsystem as Intake
import frc.robot.subsystems.leds.LEDsConstants as Constants

object LEDsSubsystem : SubsystemBase() {
	// --- LEDs ---

	private val ledStrip = LEDStrip(Constants.LENGTH, LedsMap.PWM_PORT)


	// --- LEDs State ---

	var currentMode: LEDsMode = ROBOT_DISABLED

	private val actionFinishedModeExitTimer = Timer()
	private val collectWithNoteTimer = Timer()

	private var hasCollectedNote = false


	// --- Modes Periodic Functions ---

	private fun defaultMode() {
		ledStrip.setColor(LEDStrip.LEDS_OFF)
	}

	private fun robotDisabledMode() {
		ledStrip.setColor(
			if (Robot.alliance == Alliance.Blue) RGBColor.BLUE
			else RGBColor.RED
		)
	}

	private fun actionFinishedMode() {
		ledStrip.blink(Constants.ACTION_FINISHED_MODE_BLINK_TIME)
	}

	private fun collectMode() {
		val delayPassed = collectWithNoteTimer.hasElapsed(Constants.WAIT_WITH_NOTE_DELAY)
		if (Intake.isCollectingNote && delayPassed)
			hasCollectedNote = true

		ledStrip.setColor(
			if (hasCollectedNote) RGBColor.GREEN
			else RGBColor.YELLOW
		)
	}

	private fun shootMode() {
		ledStrip.setColor(
			if (ShooterSubsystem.isWithinTolerance) RGBColor.GREEN
			else RGBColor.YELLOW
		)
	}

	private fun dynamicShootMode() {
		if (!SwerveSubsystem.isInVisionRange) {
			ledStrip.currentColor = RGBColor.ORANGE
			ledStrip.blink(Constants.DYNAMIC_SHOOTING_FAILED_BLINK_TIME)
		} else {
			shootMode()
		}
	}


	// --- LEDs Commands Control ---

	fun actionFinished(interrupted: Boolean) {
		actionFinishedModeExitTimer.restart()
		currentMode =
			if (interrupted) {
				DEFAULT
			} else {
				ledStrip.setColor(RGBColor.GREEN)
				ACTION_FINISHED
			}
	}

	fun setToDefaultMode() {
		if (currentMode != ACTION_FINISHED) currentMode = DEFAULT
	}

	fun setModeCommand(mode: LEDsMode): Command =
		instantCommand {
			currentMode = mode
			if (mode == COLLECT) {
				collectWithNoteTimer.restart()
				hasCollectedNote = false
			}
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