package com.hamosad1657.lib.units

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.math.geometry.Rotation2d

/// --- Length ---

inline val Double.meters get() = Length.fromMeters(this)
inline val Int.meters get() = Length.fromMeters(this.toDouble())

inline val Double.centimeters get() = Length.fromCentimeters(this)
inline val Int.centimeters get() = Length.fromCentimeters(this.toDouble())

inline val Double.millimeters get() = Length.fromMillimeters(this)
inline val Int.millimeters get() = Length.fromMillimeters(this.toDouble())

inline val Double.feet get() = Length.fromFeet(this)
inline val Int.feet get() = Length.fromFeet(this.toDouble())

inline val Double.inches get() = Length.fromInches(this)
inline val Int.inches get() = Length.fromInches(this.toDouble())


/// --- Angular Velocity ---

inline val Double.rpm get() = AngularVelocity.fromRpm(this)
inline val Int.rpm get() = AngularVelocity.fromRpm(this.toDouble())

inline val Double.rps get() = AngularVelocity.fromRps(this)
inline val Int.rps get() = AngularVelocity.fromRps(this.toDouble())

inline val Double.radPs get() = AngularVelocity.fromRadPs(this)
inline val Int.radPs get() = AngularVelocity.fromRadPs(this.toDouble())

inline val Double.degPs get() = AngularVelocity.fromDegPs(this)
inline val Int.degPs get() = AngularVelocity.fromDegPs(this.toDouble())


/// -- Rotation2d ---

inline val Double.degrees: Rotation2d get() = Rotation2d.fromDegrees(this)
inline val Int.degrees: Rotation2d get() = Rotation2d.fromDegrees(this.toDouble())

inline val Double.radians: Rotation2d get() = Rotation2d.fromRadians(this)
inline val Int.radians: Rotation2d get() = Rotation2d.fromRadians(this.toDouble())

inline val Double.rotations: Rotation2d get() = Rotation2d.fromRotations(this)
inline val Int.rotations: Rotation2d get() = Rotation2d.fromRotations(this.toDouble())


/// --- NeutralMode and IdleMode

fun IdleMode.toNeutralMode(): NeutralMode =
	when (this) {
		IdleMode.kCoast -> NeutralMode.Coast
		IdleMode.kBrake -> NeutralMode.Brake
	}

fun NeutralMode.toIdleMode(): IdleMode =
	when (this) {
		NeutralMode.Coast -> IdleMode.kCoast
		NeutralMode.Brake -> IdleMode.kBrake
		NeutralMode.EEPROMSetting -> throw IllegalArgumentException("can't convert `NeutralMode.EEPROMSetting` to IdleMode")
	}