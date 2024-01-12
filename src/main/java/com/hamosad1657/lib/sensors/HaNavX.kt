package com.hamosad1657.lib.sensors

import com.hamosad1657.lib.robotPrint
import com.hamosad1657.lib.robotPrintError
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.degToRad
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.*

/**
 * A wrapper class for kauailabs.navx.frc.AHRS, which adheres to WPILib's
 * coordinate system conventions.
 * @see <a href="https://en.wikipedia.org/wiki/Aircraft_principal_axes">Aircraft principal axes</a>
 * for an explanation on the different axes.
 */
// Note: this class is a wrapper over AHRS instead of a child that inherits AHRS, in order to prevent usage
// of AHRS functions directly. Instead, it provides a better API that is suited to Kotlin and our team.
class HaNavX : Sendable {
	private var navX: AHRS? = null
	private var yawOffsetDeg = 0.0

	/** The angle of the navX in range [[-180, 180]] on the Vertical-axis (perpendicular to earth, left / right rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val yawDeg get() = -navX!!.yaw.toDouble() - yawOffsetDeg

	/** The angle of the navX in range [[-PI, PI]] on the Vertical-axis (perpendicular to earth, left / right rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val yawRad get() = degToRad(yawDeg)

	/** The angle of the navX in range [[-180, 180]] on the Vertical-axis (perpendicular to earth, left / right rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val yaw: Rotation2d get() = Rotation2d.fromDegrees(yawDeg)

	/** The angle of the navX in range [[-180, 180]] on the Transverse-axis (down / up rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val pitchDeg get() = navX!!.pitch.toDouble()

	/** The angle of the navX in range [[-PI, PI]] on the Transverse-axis (down / up rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val pitchRad get() = degToRad(pitchDeg)

	/** The angle of the navX in range [[-180, 180]] on the Transverse-axis (down / up rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val pitch: Rotation2d get() = Rotation2d.fromDegrees(pitchDeg)

	/** The angle of the navX in range [[-180, 180]] on the Longitudinal-axis (lean right / lean left rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val rollDeg get() = navX!!.roll.toDouble()

	/** The angle of the navX in range [[-PI, PI]] on the Longitudinal-axis (lean right / lean left rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val rollRad get() = degToRad(rollDeg)

	/** The angle of the navX in range [[-180, 180]] on the Longitudinal-axis (lean right / lean left rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val roll: Rotation2d get() = Rotation2d.fromDegrees(rollDeg)

	/** The rate of angle change (angular velocity) of the navX in range [[-180, 180]]
	 * on the Vertical-axis (perpendicular to earth, left / right rotation).
	 *
	 * Larger value is counter-clockwise, according to WPILib's coordinate system. */
	val yawAngularVelocity get() = AngularVelocity.fromDegPs(navX!!.rate)

	/** Returns the total accumulated yaw angle (Z Axis, in degrees) reported by the sensor.
	 *
	 * NOTE: The angle is continuous, meaning it's range is beyond 360 degrees.
	 * This ensures that algorithms that wouldn't want to see a discontinuity in
	 * the gyro output as it sweeps past 0 on the second time around. */
	val accumulatedYawDeg get() = navX!!.angle

	/** Returns the total accumulated yaw angle (Z Axis) reported by the sensor.
	 *
	 * NOTE: The angle is continuous, meaning it's range is beyond 360 degrees.
	 * This ensures that algorithms that wouldn't want to see a discontinuity in
	 * the gyro output as it sweeps past 0 on the second time around. */
	val accumulatedYaw: Rotation2d get() = navX!!.rotation2d

	/** Indicates if the sensor is currently detecting motion, based upon the X and Y-axis
	 * world linear acceleration values. If the sum of the absolute values of the X and Y
	 * axis exceed a "motion threshold", the motion state is indicated. */
	val isMoving get() = navX!!.isMoving

	/** Indicates if the sensor is currently detecting yaw rotation, based upon whether
	 * the change in yaw over the last second exceeds the "Rotation Threshold." */
	val isRotating get() = navX!!.isRotating

	constructor(port: SerialPort.Port) {
		try {
			navX = AHRS(port)
			this.initialize()
		} catch (e: RuntimeException) {
			robotPrintError("Failed to initialize navX.", printStackTrace = true)
		}
	}

	constructor(port: SPI.Port) {

		try {
			navX = AHRS(port)
			this.initialize()
		} catch (e: RuntimeException) {
			robotPrintError("Failed to initialize navX.", printStackTrace = true)
		}
	}

	/** Using the onboard I2C port is [not recommended](
		https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html#onboard-i2c-causing-system-lockups) */
	constructor(port: I2C.Port) {
		try {
			navX = AHRS(port)
			this.initialize()
		} catch (e: RuntimeException) {
			robotPrintError("Failed to initialize navX.", printStackTrace = true)
		}
	}

	/** Enables logging to the RioLog & Driver Station, then waits until
	 * the navX is connected and calibrated, or 5 seconds have passed. */
	private fun initialize() {
		navX!!.enableLogging(true)

		val connectionTimeoutTimer = Timer()
		connectionTimeoutTimer.start()
		while ((!navX!!.isConnected || navX!!.isCalibrating) && !connectionTimeoutTimer.hasElapsed(TIMEOUT_SECONDS)) {
			// Wait until navX is connected and calibrated or 5 seconds have passed
		}
		connectionTimeoutTimer.stop()

		if (connectionTimeoutTimer.hasElapsed(TIMEOUT_SECONDS)) {
			robotPrintError(
				"Failed to connect to navX or navX didn't calibrate within $TIMEOUT_SECONDS seconds from startup.",
				printStackTrace = true
			)
		} else {
			robotPrint("NavX done calibrating in ${connectionTimeoutTimer.get()} seconds from startup.")
		}
	}

	/** Sets the currently facing yaw angle as zero.*/
	fun zeroYaw() {
		yawOffsetDeg = 0.0
		try {
			navX!!.zeroYaw()
			robotPrint("NavX zeroed.")
		} catch (e: RuntimeException) {
			robotPrintError("Failed to zero navX yaw.", printStackTrace = true)
		}
	}

	/** Sets the currently facing yaw angle as the zero minus the offset. */
	fun setYaw(offsetDeg: Double) {
		try {
			navX!!.zeroYaw()
			yawOffsetDeg = offsetDeg
			robotPrint("NavX offset set to $offsetDeg.")
		} catch (e: RuntimeException) {
			robotPrintError("Failed to set navX yaw offset.", printStackTrace = true)
		}
	}

	/** Sets the currently facing yaw angle as the zero minus the offset. */
	fun setYaw(offset: Rotation2d) = this.setYaw(offset.degrees)
	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("HaNavX")
		builder.addDoubleProperty("Yaw Angle Degrees", { yawDeg }, null)
		builder.addDoubleProperty("Yaw Angle Radians", { yawRad }, null)
		builder.addDoubleProperty("Pitch Angle Degrees", { pitchDeg }, null)
		builder.addDoubleProperty("Pitch Angle Radians", { pitchRad }, null)
		builder.addDoubleProperty("Roll Angle Degrees", { rollDeg }, null)
		builder.addDoubleProperty("Roll Angle Radians", { rollRad }, null)
		builder.addDoubleProperty("Yaw Angular Velocity DegPs", { yawAngularVelocity.degPs }, null)
		builder.addDoubleProperty("Yaw Offset Degrees", { yawOffsetDeg }, null)
	}

	companion object {
		private const val TIMEOUT_SECONDS = 5.0
	}
}

