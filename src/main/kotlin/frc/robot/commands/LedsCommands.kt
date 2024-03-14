package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.leds.LedsSubsystem
import frc.robot.subsystems.leds.LedsSubsystem.RGBColor
import frc.robot.subsystems.loader.LoaderSubsystem
import frc.robot.subsystems.shooter.ShooterSubsystem

fun LedsSubsystem.setColorCommand(color: RGBColor): Command = withName("set color") {
	LedsSubsystem.runOnce { setColor(color) } andThen LedsSubsystem.run {}
}

fun LedsSubsystem.blinkCommand(color: RGBColor, blinkTime: Seconds): Command = withName("blink") {
	(setColorCommand(color) withTimeout blinkTime) andThen
		(setColorCommand(RGBColor.DARK) withTimeout blinkTime)
			.repeatedly()
}

fun LedsSubsystem.intakeModeCommand(): Command = withName("intake mode") {
	run {
		if (IntakeSubsystem.isBottomMotorUnderLoad) {
			blinkCommand(RGBColor.MAGENTA, 0.1).asProxy().schedule()
		} else {
			setColorCommand(RGBColor.WHITE).asProxy().schedule()
		}
	}
}

fun LedsSubsystem.shootingModeCommand(): Command = withName("shooting mode") {
	RunCommand({
		if (ShooterSubsystem.isWithinTolerance) {
			blinkCommand(RGBColor.MAGENTA, 0.1).schedule()
		} else {
			setColorCommand(RGBColor.WHITE).schedule()
		}
	})
}

fun LedsSubsystem.blinkReadyCommand(): Command = withName("blink ready") {
	blinkCommand(RGBColor.PURE_GREEN, 0.2)
}

fun LedsSubsystem.ledsDefaultCommand(): Command = withName("default") {
	RunCommand({
		if (LoaderSubsystem.isNoteDetected) {
			setColorCommand(RGBColor.TEAM_GREEN).schedule()
		} else {
			setColorCommand(RGBColor.DARK).schedule()
		}
	})
}