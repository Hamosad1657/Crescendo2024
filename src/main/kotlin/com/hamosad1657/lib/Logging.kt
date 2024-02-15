package com.hamosad1657.lib

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.wpilibj.DriverStation

fun robotPrint(message: Any?, printStackTrace: Boolean = false) =
	DriverStation.reportWarning(message.toString(), printStackTrace)

fun robotPrintError(message: Any?, printStackTrace: Boolean = false) =
	DriverStation.reportError(message.toString(), printStackTrace)

val robotAlliance: DriverStation.Alliance
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

val SWERVE_MODULE_NAMES = arrayOf("FrontLeft", "FrontRight", "BackLeft", "BackRight")

fun SwerveModuleState.toSendable(moduleIndex: Int, prefix: String = "") =
	Sendable { builder ->
		val moduleName = SWERVE_MODULE_NAMES[moduleIndex]
		builder.addDoubleProperty("$prefix$moduleName/MPS", { speedMetersPerSecond }, null)
		builder.addDoubleProperty("$prefix$moduleName/Angle", { angle.degrees }, null)
	}

fun Pose2d.toSendable(prefix: String = "") =
	Sendable { builder ->
		builder.addStringProperty("${prefix}/Pose", { "x=$x, y=$y, theta=${rotation.degrees}" }, null)
	}