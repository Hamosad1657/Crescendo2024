package frc.robot.subsystems.climbing

import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.PercentOutput
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkFlex
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Climbing as ClimbingMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem : SubsystemBase() {
	// --- Motors & Sensors ---

	private val leftMainMotor =
		HaSparkFlex(ClimbingMap.LEFT_FRONT_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configMainMotor(inverted = true)
		}
	private val leftSecondaryMotor =
		HaSparkFlex(ClimbingMap.LEFT_BACK_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configSecondaryMotor(leftMainMotor)
		}

	private val rightMainMotor =
		HaSparkFlex(ClimbingMap.RIGHT_FRONT_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configMainMotor(inverted = false)
		}
	private val rightSecondaryMotor =
		HaSparkFlex(ClimbingMap.RIGHT_BACK_MOTOR_ID).apply {
			restoreFactoryDefaults()
			configSecondaryMotor(rightMainMotor)
		}

	private val leftEncoder = leftMainMotor.encoder
	private val rightEncoder = rightMainMotor.encoder

	// --- Motors Configuration ---

	private fun CANSparkFlex.configMainMotor(inverted: Boolean) =
		apply {
			this.inverted = inverted
			idleMode = IdleMode.kBrake
			setSmartCurrentLimit(Constants.SMART_CURRENT_LIMIT)
		}

	private fun CANSparkFlex.configSecondaryMotor(mainMotor: CANSparkFlex) =
		apply {
			inverted = false
			idleMode = IdleMode.kBrake
			setSmartCurrentLimit(Constants.SMART_CURRENT_LIMIT)
			follow(mainMotor, true)
		}

	var idleMode = IdleMode.kBrake
		set(value) {
			leftMainMotor.idleMode = value
			leftSecondaryMotor.idleMode = value
			rightMainMotor.idleMode = value
			rightSecondaryMotor.idleMode = value
			field = value
		}

	// --- Motors Control ---

	fun set(output: PercentOutput) {
		setLeft(output)
		setRight(output)
	}

	fun setLeft(output: PercentOutput) {
		leftMainMotor.set(output)
	}

	fun setRight(output: PercentOutput) {
		rightMainMotor.set(output)
	}

	fun stopMotors() {
		stopLeft()
		stopRight()
	}

	fun stopLeft() {
		leftMainMotor.stopMotor()
	}

	fun stopRight() {
		rightMainMotor.stopMotor()
	}


	// --- Periodic & Telemetry ---

	override fun periodic() {}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("Subsystem")
		builder.addStringProperty("Command", { currentCommand?.name ?: "none" }, null)
		builder.addDoubleProperty("Left output", { leftMainMotor.get() }, null)
		builder.addDoubleProperty("Right output", { rightMainMotor.get() }, null)
		builder.addDoubleProperty("Left position", { leftEncoder.position }, null)
		builder.addDoubleProperty("Right position", { rightEncoder.position }, null)
	}
}
