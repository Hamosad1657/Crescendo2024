package com.hamosad1657.lib

import edu.wpi.first.wpilibj.DriverStation

fun robotPrint(message: Any?, printStackTrace: Boolean = false) =
    DriverStation.reportWarning(message.toString(), printStackTrace)

fun robotPrintError(message: Any?, printStackTrace: Boolean = false) =
    DriverStation.reportError(message.toString(), printStackTrace)

val alliance: DriverStation.Alliance
    get() {
        if (DriverStation.getAlliance().isEmpty) {
            throw NoSuchElementException("Alliance invalid or can't fetch alliance from Driver Station")
        }
        return DriverStation.getAlliance().get()
    }

val driverStationID: Int
    get() {
        if (DriverStation.getLocation().isEmpty) {
            throw NoSuchElementException("Station ID invalid or can't fetch from Driver Station")
        }
        return DriverStation.getLocation().asInt
    }

enum class Telemetry {
    Testing, Simulation, Competition;
}