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
import frc.robot.RobotMap.LEDs as LEDsMap
import frc.robot.subsystems.intake.IntakeSubsystem as Intake
import frc.robot.subsystems.leds.LEDsConstants as Constants
import frc.robot.subsystems.shooter.ShooterSubsystem as Shooter
import frc.robot.subsystems.swerve.SwerveSubsystem as Swerve

object LEDsSubsystem : SubsystemBase() {
	// --- LEDs ---

	private val ledStrip = LEDStrip(Constants.LENGTH, LEDsMap.PWM_PORT)


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
		ledStrip.currentColor = if (Robot.alliance == Alliance.Blue) RGBColor.BLUE else RGBColor.RED
		if (Robot.isTesting) ledStrip.apply { setColorAlternating(currentColor) }
		else ledStrip.apply { setColor(currentColor) }
	}

	private fun actionFinishedMode() {
		ledStrip.blink(Constants.ACTION_FINISHED_MODE_BLINK_TIME)
	}

	private fun actionFailingMode() {
		ledStrip.currentColor = RGBColor.ORANGE
		ledStrip.blink(Constants.ACTION_FAILING_BLINK_TIME)
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
			if (Shooter.isWithinTolerance) RGBColor.GREEN
			else RGBColor.YELLOW
		)
	}

	private fun dynamicShootMode() {
		if (!Swerve.isInVisionRange) actionFailingMode()
		else shootMode()
	}

	private fun teleopShootMode() {
		ledStrip.setColor(RGBColor.MAGENTA)
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

			ACTION_FAILING -> actionFailingMode()
			COLLECT -> collectMode()
			SHOOT -> shootMode()
			DYNAMIC_SHOOT -> dynamicShootMode()
			TELEOP_SHOOTER -> teleopShootMode()
			DEFAULT -> defaultMode()
			ROBOT_DISABLED -> robotDisabledMode()
		}
	}
}