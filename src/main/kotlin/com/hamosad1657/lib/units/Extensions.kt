package com.hamosad1657.lib.units

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.absoluteValue

// --- Length ---

inline val Number.meters get() = Length.fromMeters(this.toDouble())
inline val Number.centimeters get() = Length.fromCentimeters(this.toDouble())
inline val Number.millimeters get() = Length.fromMillimeters(this.toDouble())
inline val Number.feet get() = Length.fromFeet(this.toDouble())
inline val Number.inches get() = Length.fromInches(this.toDouble())


// --- Angular Velocity ---

inline val Number.rpm get() = AngularVelocity.fromRpm(this.toDouble())
inline val Number.rps get() = AngularVelocity.fromRps(this.toDouble())
inline val Number.radPs get() = AngularVelocity.fromRadPs(this.toDouble())
inline val Number.degPs get() = AngularVelocity.fromDegPs(this.toDouble())


// -- Rotation2d ---

inline val Number.degrees: Rotation2d get() = Rotation2d.fromDegrees(this.toDouble())
inline val Number.radians: Rotation2d get() = Rotation2d.fromRadians(this.toDouble())
inline val Number.rotations: Rotation2d get() = Rotation2d.fromRotations(this.toDouble())

inline val Rotation2d.absoluteValue: Rotation2d get() = Rotation2d.fromRotations(this.rotations.absoluteValue)

infix fun Rotation2d.plus(other: Rotation2d) = (this.degrees + other.degrees).degrees
infix fun Rotation2d.minus(other: Rotation2d) = (this.degrees - other.degrees).degrees

// --- NeutralMode and IdleMode

fun IdleMode.toNeutralModeValue(): NeutralModeValue =
	when (this) {
		IdleMode.kCoast -> NeutralModeValue.Coast
		IdleMode.kBrake -> NeutralModeValue.Brake
	}

fun IdleMode.toNeutralMode(): NeutralMode =
	when (this) {
		IdleMode.kCoast -> NeutralMode.Coast
		IdleMode.kBrake -> NeutralMode.Brake
	}
