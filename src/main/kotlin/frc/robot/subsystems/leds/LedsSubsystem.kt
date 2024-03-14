package frc.robot.subsystems.leds

import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LedsConstants.EXIT_BLINK_DONE_MODE_TIMEOUT
import frc.robot.subsystems.leds.LedsConstants.LEDsMode
import frc.robot.subsystems.leds.LedsConstants.LEDsMode.*
import frc.robot.subsystems.leds.LedsConstants.RGBColor
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.leds.LedsConstants as Constants

object LedsSubsystem : SubsystemBase() {
	var currentMode: LEDsMode = DEFAULT
		private set

	private val blinkTimer = Timer()
	private val exitBlinkDoneModeTimer = Timer()

	private val ledsBuffer = AddressableLEDBuffer(Constants.LENGTH)
	private val ledStrip = AddressableLED(RobotMap.Leds.PWM_PORT).apply {
		setData(ledsBuffer)
		start()
	}

	val areLedsOn: Boolean
		get() =
			(ledsBuffer.getRed(0) != 0) or
				(ledsBuffer.getGreen(0) != 0) or
				(ledsBuffer.getBlue(0) != 0)

	private fun setColor(color: RGBColor) {
		for (i in 0..Constants.LENGTH) {
			ledsBuffer.setRGB(i, color.red, color.green, color.blue)
		}
		ledStrip.setData(ledsBuffer)
	}

	private fun toggleLeds(colorIfOff: RGBColor) {
		if (areLedsOn) {
			setColor(RGBColor.DARK)
		} else {
			setColor(colorIfOff)
		}
	}

	/** Should be called periodically. */
	private fun blink(color: RGBColor, blinkTime: Seconds) {
		blinkTimer.start()
		if (blinkTimer.hasElapsed(blinkTime)) {
			toggleLeds(color)
			blinkTimer.restart()
		}
	}

	private fun collectMode() {
		if (IntakeSubsystem.isBottomMotorUnderLoad) {
			setColor(RGBColor.MAGENTA)
		} else {
			setColor(RGBColor.WHITE)
		}
	}

	private fun shootMode() {
		if (ShooterSubsystem.isWithinTolerance) {
			setColor(RGBColor.MAGENTA)
		} else {
			setColor(RGBColor.WHITE)
		}
	}

	private fun defaultMode() {
		if (LoaderSubsystem.isNoteDetected) {
			setColor(RGBColor.TEAM_GREEN)
		} else {
			setColor(RGBColor.DARK)
		}
	}

	private fun blinkDoneMode() {
		blink(RGBColor.PURE_GREEN, Constants.BLINK_DONE_TIME)
	}

	fun setModeCommand(mode: LEDsMode): Command = InstantCommand({
		currentMode = mode
	})

	fun exitAMode(interrupted: Boolean) {
		if (interrupted) {
			currentMode = DEFAULT
		} else {
			currentMode = BLINK_DONE
		}
	}

	override fun periodic() {
		when (currentMode) {
			COLLECT -> {
				collectMode()
			}

			SHOOT -> {
				shootMode()
			}

			DEFAULT -> {
				defaultMode()
			}

			BLINK_DONE -> {
				exitBlinkDoneModeTimer.start()
				blinkDoneMode()
				if (exitBlinkDoneModeTimer.hasElapsed(EXIT_BLINK_DONE_MODE_TIMEOUT)) {
					currentMode = DEFAULT
					exitBlinkDoneModeTimer.stop()
				}
			}
		}
	}
}