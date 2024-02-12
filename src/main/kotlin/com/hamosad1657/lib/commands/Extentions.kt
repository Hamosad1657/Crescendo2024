package com.hamosad1657.lib.commands

import edu.wpi.first.wpilibj2.command.*

fun waitUntil(until: () -> Boolean) = WaitUntilCommand(until)

infix fun Command.until(condition: () -> Boolean): Command = this.until(condition)
infix fun Command.andThen(next: Command): Command = this.andThen(next)
infix fun Command.finallyDo(end: (interrupted: Boolean) -> Unit): Command = this.finallyDo(end)

infix fun Command.alongWith(parallel: Command): Command = this.alongWith(parallel)
infix fun Command.raceWith(parallel: Command): Command = this.raceWith(parallel)

infix fun Command.withTimeout(seconds: Double): Command = this.withTimeout(seconds)

/**
 * Good for multi-subsystem commands.
 * For single-subsystem commands, use [SubsystemBase.withName].
 */
infix fun Command.withName(commandName: String): Command = this.withName(commandName)

fun withName(commandName: String, commandSupplier: () -> Command): Command =
	commandSupplier().also { it.name = commandName }

/**
 * Good for single-subsystem commands.
 * Appends the name of the subsystem to the String in [commandName : subsystemName] format.
 *
 * For multi-subsystem commands, use [Command.withName].
 */
fun SubsystemBase.withName(commandName: String, commandSupplier: () -> Command): Command =
	commandSupplier().also { it.name = "$commandName : ${this.name}" }