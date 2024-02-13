package com.hamosad1657.lib.sensors

import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.Encoder

/**
 * A sendable quadrature encoder, wrapping WPILib's Encoder class.
 * For an explanation on quadrature encoders, read here:
 * [...](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html)
 */
class HaEncoder : Sendable {
	private val encoder: Encoder
	private val pulsesPerRev: Double
	private var wheelRadiusM = 0.0

	/**
	 * Construct a new HaEncoder. It starts counting immediately.
	 *
	 * @param channelA         - The digital input port on the RoboRIO (onboard or
	 * MXP) that the encoder's channel A is wired to.
	 * @param channelB         - The digital input port that the encoder's channel B
	 * is wired to.
	 * @param pulsesPerRev     - The encoder's pulses per revolution.
	 * @param reverseDirection - Whether to invert the speed and distance.
	 */
	constructor(channelA: Int, channelB: Int, pulsesPerRev: Int, reverseDirection: Boolean) {
		encoder = Encoder(channelA, channelB, reverseDirection)
		this.pulsesPerRev = pulsesPerRev.toDouble()
		encoder.distancePerPulse = this.pulsesPerRev
	}

	/**
	 * Construct a new HaEncoder. It starts counting immediately.
	 *
	 * @param channelA     - The digital input port on the RoboRIO (onboard or
	 * MXP) that the encoder's channel A is wired to.
	 * @param channelB     - The digital input port that the encoder's channel B
	 * is wired to.
	 * @param pulsesPerRev - The encoder's pulses per revolution.
	 */
	constructor(channelA: Int, channelB: Int, pulsesPerRev: Int) {
		encoder = Encoder(channelA, channelB)
		this.pulsesPerRev = pulsesPerRev.toDouble()
		encoder.distancePerPulse = this.pulsesPerRev
	}

	/**
	 * @param gearRatio - The gear reduction ratio between the encoder shaft and the
	 * wheel/mechanism. For example, for a 4:1 speed reduction pass
	 * 0.25, not 4. If not set, it defaults to 1.
	 */
	fun setGearRatio(gearRatio: Double) {
		encoder.distancePerPulse = pulsesPerRev * gearRatio
	}

	/**
	 * @param wheelRadiusM - The radius of the wheel in meters, to use with
	 * getDistanceMeters and getSpeedMPS.
	 */
	fun setWheelRadius(wheelRadiusM: Double) {
		this.wheelRadiusM = wheelRadiusM
	}

	private val rotations: Double
		/**
		 * @return The rotations traveled since last reset, accounting the gear ratio as
		 * configured in setGearRatio().
		 */
		get() = encoder.distance
	val degrees: Double
		/**
		 * @return The degrees traveled since last reset, accounting the gear ratio as
		 * configured in setGearRatio().
		 */
		get() = rotations * 360
	val distanceMeters: Double
		/**
		 * @return The distance traveled in meters since last reset, as configured in
		 * setWheelRadius, and accounting the gear ratio as configured in
		 * setGearRatio().
		 */
		get() = rotations * wheelRadiusM * 2 * Math.PI
	private val speedRPM: Double
		/**
		 * @return The speed in rotations per minute, accounting the gear ratio as
		 * configured in setGearRatio().
		 */
		get() = encoder.rate / 60
	val speedDegPS: Double
		/**
		 * @return The speed in degrees per second, accounting the gear ratio as
		 * configured in setGearRatio().
		 */
		get() = encoder.rate * 360
	val speedMPS: Double
		/**
		 * @return The wheel speed in meters per second, as configured in
		 * setWheelRadius, and accounting the gear ratio as configured in
		 * setGearRatio().
		 */
		get() = encoder.rate * 2 * Math.PI * wheelRadiusM

	/**
	 * Sets the distance/rotations count to zero.
	 */
	fun reset() {
		encoder.reset()
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("HaEncoder")
		builder.addDoubleProperty("Rotations", { rotations }, null)
		builder.addDoubleProperty("RPM", { speedRPM }, null)
	}
}
