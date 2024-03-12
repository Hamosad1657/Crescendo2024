package frc.robot.subsystems.stabilizers

import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.Amps
import com.hamosad1657.lib.units.Volts
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkFlex
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Climbing.Stabilizers as StabilizersMap
import frc.robot.subsystems.stabilizers.StabilizersConstants as Constants

object StabilizersSubsystem : SubsystemBase() {
	// --- Motors ---

	private val leftMotor =
		HaSparkFlex(StabilizersMap.LEFT_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configMotor(inverted = false)
		}

	private val rightMotor =
		HaSparkFlex(StabilizersMap.RIGHT_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configMotor(inverted = false)
		}


	// --- Motors Configuration ---

	private fun CANSparkFlex.configMotor(inverted: Boolean) {
		idleMode = IdleMode.kBrake
		setSmartCurrentLimit(Constants.SMART_CURRENT_LIMIT)
	}

	var idleMode = IdleMode.kBrake
		set(value) {
			leftMotor.idleMode = value
			rightMotor.idleMode = value
			field = value
		}


	// --- State Getters ---

	val leftMotorCurrent: Amps get() = leftMotor.outputCurrent.toInt()
	val rightMotorCurrent: Amps get() = leftMotor.outputCurrent.toInt()

	val leftAtLimit get() = leftMotorCurrent > Constants.MOTOR_STALL_CURRENT
	val rightAtLimit get() = rightMotorCurrent > Constants.MOTOR_STALL_CURRENT


	// --- Motors Control ---

	fun setLeft(output: Volts) {
		leftMotor.setVoltage(output)
	}

	fun setRight(output: Volts) {
		rightMotor.setVoltage(output)
	}

	fun stopMotors() {
		stopLeft()
		stopRight()
	}

	fun stopLeft() {
		leftMotor.stopMotor()
	}

	fun stopRight() {
		rightMotor.stopMotor()
	}


	// --- Periodic & Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("Subsystem")
		builder.addStringProperty("Command", { currentCommand?.name ?: "none" }, null)
		builder.addIntegerProperty("Left current", { leftMotorCurrent.toLong() }, null)
		builder.addIntegerProperty("Right current", { rightMotorCurrent.toLong() }, null)
	}
}