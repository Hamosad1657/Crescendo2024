package frc.robot.subsystems.leds

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.RobotMap
import frc.robot.subsystems.leds.LedsConstants as Constants

object LedsSubsystem : SubsystemBase() {
	data class RGBColor(val red: Int, val green: Int, val blue: Int) {
		companion object {
			val TEAM_GREEN = RGBColor(22, 87, 0)
			val RED = RGBColor(255, 0, 0)
			val BLUE = RGBColor(0, 0, 255)
			val YELLOW = RGBColor(255, 255, 0)
			val CYAN = RGBColor(0, 255, 255)
			val MAGENTA = RGBColor(255, 0, 255)
			val DARK = RGBColor(0, 0, 0)
		}
	}

	val ledsBuffer = AddressableLEDBuffer(Constants.LENGTH)
	val ledStrip = AddressableLED(RobotMap.Leds.PWM_PORT).apply {
		setData(ledsBuffer)
		start()
	}

	fun setColor(color: RGBColor) {
		for (i in 0..Constants.LENGTH) {
			ledsBuffer.setRGB(i, color.red, color.green, color.blue)
		}
		ledStrip.setData(ledsBuffer)
	}

	fun setColorCommand(color: RGBColor): Command = withName("set color") {
		runOnce { setColor(color) }
	}

	fun blinkCommand(color: RGBColor, blinkTime: Seconds): Command = withName("blink") {
		(setColorCommand(color) alongWith WaitCommand(blinkTime)) andThen
			(setColorCommand(RGBColor.DARK) alongWith WaitCommand(blinkTime))
				.repeatedly()
	}
}