package com.hamosad1657.lib.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand

fun waitUntil(until: () -> Boolean) = WaitUntilCommand(until)

infix fun Command.andThen(next: Command): Command = this.andThen(next)
infix fun Command.until(condition: () -> Boolean): Command = this.until(condition)
infix fun Command.alongWith(parallel: Command): Command = this.alongWith(parallel)
infix fun Command.finallyDo(end: (interrupted: Boolean) -> Unit): Command = this.finallyDo(end)

fun SubsystemBase.withName(commandName: String, commandSupplier: () -> Command): Command =
    commandSupplier().also { it.name = "$commandName : $name" }