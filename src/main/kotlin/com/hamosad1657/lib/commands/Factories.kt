package com.hamosad1657.lib.commands

import com.hamosad1657.lib.units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand

fun wait(duration: Seconds) = WaitCommand(duration)
fun waitUntil(until: () -> Boolean) = WaitUntilCommand(until)
fun instantCommand(toRun: () -> Unit) = InstantCommand(toRun)

/** THIS COMMAND DOES NOT REQUIRE ANY SUBSYSTEMS. */
val (() -> Unit).asInstantCommand: Command get() = InstantCommand(this)
