package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem
import frc.robot.subsystems.leds.LedsConstants as Constants

object LedsSubsystem : SubsystemBase() {
	data class RGBColor(val red: Int, val green: Int, val blue: Int) {
		companion object {
			val TEAM_GREEN = RGBColor(22, 87, 0)
			val PURE_GREEN = RGBColor(0, 255, 0)
			val WHITE = RGBColor(255, 255, 255)
			val RED = RGBColor(255, 0, 0)
			val BLUE = RGBColor(0, 0, 255)
			val YELLOW = RGBColor(255, 255, 0)
			val CYAN = RGBColor(0, 255, 255)
			val MAGENTA = RGBColor(255, 0, 255)
			val DARK = RGBColor(0, 0, 0)
		}
	}

	enum class LEDsMode {
		COLLECT, SHOOT, BLINK_READY, DEFAULT,
	}

	private val ledsBuffer = AddressableLEDBuffer(Constants.LENGTH)
	private val ledStrip = AddressableLED(RobotMap.Leds.PWM_PORT).apply {
		setData(ledsBuffer)
		start()
	}

	fun setColor(color: RGBColor) {
		for (i in 0..Constants.LENGTH) {
			ledsBuffer.setRGB(i, color.red, color.green, color.blue)
		}
		ledStrip.setData(ledsBuffer)
	}

	private fun collectMode() {
		if (IntakeSubsystem.isBottomMotorUnderLoad) {
			setColor(RGBColor.MAGENTA)
		} else {
			setColor(RGBColor.WHITE)
		}
	}

	private fun shooterMode() {
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
}