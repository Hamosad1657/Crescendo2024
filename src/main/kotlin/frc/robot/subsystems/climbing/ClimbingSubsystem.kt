package frc.robot.subsystems.climbing

import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Rotations
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkFlex
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Climbing as ClimbingMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem : SubsystemBase() {

	// --- Motors and Sensors ---

	private val leftMainMotor = HaSparkFlex(ClimbingMap.LEFT_FRONT_MOTOR_ID).apply {
		restoreFactoryDefaults()
		// TODO: Verify positive output raises climber
		inverted = true
		idleMode = IdleMode.kBrake
		setSmartCurrentLimit(Constants.SMART_CURRENT_LIMIT)
	}
	private val leftSecondaryMotor = HaSparkFlex(ClimbingMap.LEFT_BACK_MOTOR_ID).apply {
		configSecondaryMotor(leftMainMotor)
	}

	private val rightMainMotor = HaSparkFlex(ClimbingMap.RIGHT_FRONT_MOTOR_ID).apply {
		restoreFactoryDefaults()
		// TODO: Verify positive output raises climber
		inverted = false
		idleMode = IdleMode.kBrake
		setSmartCurrentLimit(Constants.SMART_CURRENT_LIMIT)
	}
	private val rightSecondaryMotor = HaSparkFlex(ClimbingMap.RIGHT_BACK_MOTOR_ID).apply {
		configSecondaryMotor(rightMainMotor)
	}

	private val leftEncoder = leftMainMotor.encoder
	private val rightEncoder = rightMainMotor.encoder

	private val leftOpenedLimitSwitch = DigitalInput(ClimbingMap.LEFT_OPENED_LIMIT_CHANNEL)
	private val leftClosedLimitSwitch = DigitalInput(ClimbingMap.LEFT_CLOSED_LIMIT_CHANNEL)
	private val rightOpenedLimitSwitch = DigitalInput(ClimbingMap.RIGHT_OPENED_LIMIT_CHANNEL)
	private val rightClosedLimitSwitch = DigitalInput(ClimbingMap.RIGHT_CLOSED_LIMIT_CHANNEL)

	private val leftTrapSwitch = DigitalInput(ClimbingMap.LEFT_TRAP_SWITCH)
	private val rightTrapSwitch = DigitalInput(ClimbingMap.RIGHT_TRAP_SWITCH)

	private var lastSetpoint: Rotations = 0.0


	// --- Motors Configuration ---

	private fun CANSparkFlex.configSecondaryMotor(mainMotor: CANSparkFlex) =
		apply {
			restoreFactoryDefaults()
			idleMode = IdleMode.kBrake
			setSmartCurrentLimit(Constants.SMART_CURRENT_LIMIT)
			// TODO: Check if follower motor should oppose master motor or not
			follow(mainMotor, true)
		}

	// --- Switches ---

	// TODO: Check if switches are wired normally open or normally closed.

	val isLeftAtClosedLimit get() = leftClosedLimitSwitch.get()
	val isLeftAtOpenedLimit get() = leftOpenedLimitSwitch.get()
	val isRightAtClosedLimit get() = false
	val isRightAtOpenedLimit get() = false

	val isLeftTrapSwitchPressed get() = !leftTrapSwitch.get()
	val isRightTrapSwitchPressed get() = !rightTrapSwitch.get()
	val areBothTrapSwitchesPressed get() = isLeftTrapSwitchPressed && isRightTrapSwitchPressed

	val isAtClosedLimit get() = isLeftAtClosedLimit && isRightAtClosedLimit
	val isAtOpenedLimit get() = isLeftAtOpenedLimit && isRightAtOpenedLimit


	// --- Motors Control ---

	val currentPosition: Rotations
		get() = leftMainMotor.encoder.position

	fun set(output: PercentOutput) {
		setLeft(output)
		setRight(output)
	}

	fun setLeft(output: PercentOutput) {
		if (isLeftAtOpenedLimit && output < 0.0) leftMainMotor.set(0.0)
		else if (isLeftAtClosedLimit && output > 0.0) leftMainMotor.set(Constants.KEEP_CLOSED_OUTPUT)
		else leftMainMotor.set(output)
	}

	fun setRight(output: PercentOutput) {
		if (isRightAtOpenedLimit && output < 0.0) rightMainMotor.set(0.0)
		else if (isRightAtClosedLimit && output > 0.0) rightMainMotor.set(Constants.KEEP_CLOSED_OUTPUT)
		else rightMainMotor.set(output)
	}

	fun stopMotors() {
		leftMainMotor.stopMotor()
		rightMainMotor.stopMotor()
	}


	override fun periodic() {
		if (isLeftAtClosedLimit) leftEncoder.position = 0.0
		if (isRightAtClosedLimit) rightEncoder.position = 0.0
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("Subsystem")
		builder.addStringProperty("Command", { currentCommand?.name ?: "none" }, null)
		builder.addBooleanProperty("Left at opened limit", { isLeftAtOpenedLimit }, null)
		builder.addBooleanProperty("Left at closed limit", { isLeftAtClosedLimit }, null)
		builder.addBooleanProperty("Right at opened limit", { isRightAtOpenedLimit }, null)
		builder.addBooleanProperty("Right at closed limit", { isRightAtClosedLimit }, null)
		builder.addDoubleProperty("Left output", { leftMainMotor.get() }, null)
		builder.addDoubleProperty("Right output", { rightMainMotor.get() }, null)
		builder.addDoubleProperty("Position rotations", { currentPosition }, null)
		builder.addBooleanProperty("Left trap switch pressed", { isLeftTrapSwitchPressed }, null)
		builder.addBooleanProperty("Right trap switch pressed", { isRightTrapSwitchPressed }, null)

	}
}
