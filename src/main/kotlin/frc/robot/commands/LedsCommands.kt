package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.leds.LedsSubsystem
import frc.robot.subsystems.leds.LedsSubsystem.RGBColor

fun setColorCommand(color: RGBColor): Command = withName("set color") {
	LedsSubsystem.runOnce { LedsSubsystem.setColor(color) } andThen LedsSubsystem.run {}
}

fun blinkCommand(color: RGBColor, blinkTime: Seconds): Command = withName("blink") {
	(setColorCommand(color) withTimeout blinkTime) andThen
		(setColorCommand(RGBColor.DARK) withTimeout blinkTime)
			.repeatedly()
}