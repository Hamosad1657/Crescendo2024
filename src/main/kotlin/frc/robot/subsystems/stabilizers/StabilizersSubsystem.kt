package frc.robot.subsystems.stabilizers

import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.Volts
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkFlex
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Climbing.Stabilizers as StabilizersMap
import frc.robot.subsystems.stabilizers.StabilizersConstants as Constants

object StabilizersSubsystem : SubsystemBase() {
	// --- Motors ---

	private val leftMotor =
		HaSparkFlex(StabilizersMap.LEFT_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configMotor(inverted = true)
		}

	private val rightMotor =
		HaSparkFlex(StabilizersMap.RIGHT_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configMotor(inverted = false)
		}


	// --- Motors Configuration ---

	private fun CANSparkFlex.configMotor(inverted: Boolean) {
		this.inverted = inverted
		idleMode = IdleMode.kBrake
		setSmartCurrentLimit(Constants.SMART_CURRENT_LIMIT)
	}

	var idleMode = IdleMode.kBrake
		set(value) {
			leftMotor.idleMode = value
			rightMotor.idleMode = value
			field = value
		}


	// --- Motors Control ---

	fun set(output: Volts) {
		setLeft(output)
		setRight(output)
	}

	fun setLeft(output: Volts) {
		leftMotor.setVoltage(output)
	}

	fun setRight(output: Volts) {
		rightMotor.setVoltage(output)
	}

	fun stopMotors() {
		leftMotor.stopMotor()
		rightMotor.stopMotor()
	}
}