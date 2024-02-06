package frc.robot.subsystems.climbing

import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.units.FractionalOutput
import com.hamosad1657.lib.units.Rotations
import com.hamosad1657.lib.units.toIdleMode
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.climbing.ClimbingConstants.MAX_POSSIBLE_POSITION
import frc.robot.subsystems.climbing.ClimbingConstants.STAY_FOLDED_OUTPUT
import frc.robot.subsystems.climbing.ClimbingConstants.WEIGHT_BEARING_PID_GAINS
import frc.robot.RobotMap.Climbing as ClimbingMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem : SubsystemBase() {

	// --- Motors and sensors ---

	private val leftMainMotor = CANSparkFlex(ClimbingMap.LEFT_MAIN_MOTOR_ID, MotorType.kBrushless)
		.apply { configMainMotor() }

	private val leftSecondaryMotor = CANSparkFlex(ClimbingMap.LEFT_SECONDARY_MOTOR_ID, MotorType.kBrushless)
		.apply { configSecondaryMotor(leftMainMotor) }

	private val rightMainMotor = CANSparkFlex(ClimbingMap.RIGHT_MAIN_MOTOR_ID, MotorType.kBrushless)
		.apply { configMainMotor() }

	private val rightSecondaryMotor = CANSparkFlex(ClimbingMap.RIGHT_SECONDARY_MOTOR_ID, MotorType.kBrushless)
		.apply { configSecondaryMotor(rightMainMotor) }

	private val pidController = WEIGHT_BEARING_PID_GAINS.toPIDController()
	private var feedForwardPercentOutput = 0.0

	private val leftEncoder = leftMainMotor.encoder
	private val rightEncoder = rightMainMotor.encoder

	private val leftOpenedLimitSwitch = DigitalInput(ClimbingMap.LEFT_OPENED_LIMIT_CHANNEL)
	private val leftClosedLimitSwitch = DigitalInput(ClimbingMap.LEFT_CLOSED_LIMIT_CHANNEL)
	private val rightOpenedLimitSwitch = DigitalInput(ClimbingMap.RIGHT_OPENED_LIMIT_CHANNEL)
	private val rightClosedLimitSwitch = DigitalInput(ClimbingMap.RIGHT_CLOSED_LIMIT_CHANNEL)

	private var lastSetpoint: Rotations = 0.0

	// --- Motors Configuration ---

	private fun CANSparkFlex.configMainMotor() =
		apply {
			// TODO: Verify positive output raises climber
			inverted = false
		}

	private fun CANSparkFlex.configSecondaryMotor(mainMotor: CANSparkFlex) =
		apply {
			// TODO: Check if follower motor should oppose master motor or not
			this.follow(mainMotor, false)
		}


	// --- Motors Properties ---

	var neutralMode = NeutralModeValue.Brake
		set(value) {
			leftMainMotor.setIdleMode(value.toIdleMode())
			leftSecondaryMotor.setIdleMode(value.toIdleMode())
			rightMainMotor.setIdleMode(value.toIdleMode())
			rightSecondaryMotor.setIdleMode(value.toIdleMode())
			field = value
		}

	val closedLoopError: Rotations
		get() = lastSetpoint - currentPosition

	val isWithinTolerance: Boolean
		get() = closedLoopError <= Constants.SETPOINT_TOLERANCE


	// --- Switches ---

	// TODO: Check if switches are wired normally open or normally closed.

	val isLeftAtClosedLimit: Boolean
		get() = leftClosedLimitSwitch.get()

	val isRightAtClosedLimit: Boolean
		get() = rightClosedLimitSwitch.get()

	val isLeftAtOpenedLimit: Boolean
		get() {
			return leftOpenedLimitSwitch.get() || currentPosition >= MAX_POSSIBLE_POSITION
		}

	val isRightAtOpenedLimit: Boolean
		get() {
			return rightOpenedLimitSwitch.get() || currentPosition >= MAX_POSSIBLE_POSITION
		}

	val isAtOpenedLimit: Boolean
		get() = isLeftAtOpenedLimit || isRightAtOpenedLimit

	val isAtClosedLimit: Boolean
		get() = isLeftAtClosedLimit || isRightAtClosedLimit


	// --- Motors Control ---

	fun configPIDF(gains: PIDGains) {
		feedForwardPercentOutput = gains.kFF()
		pidController.configPID(gains)
	}

	val currentPosition: Rotations
		get() = leftMainMotor.encoder.position

	fun setPositionSetpoint(newSetpoint: Rotations) {
		if (lastSetpoint != newSetpoint) {
			pidController.reset()
		}
		val controlEffort = pidController.calculate(currentPosition, newSetpoint) + feedForwardPercentOutput
		set(controlEffort)
		lastSetpoint = newSetpoint
	}

	fun increasePositionSetpointBy(desiredChangeInPosition: Rotations) {
		setPositionSetpoint(currentPosition + desiredChangeInPosition)
	}

	fun set(output: FractionalOutput) {
		setLeft(output)
		setRight(output)
	}

	private fun PIDController.configPID(gains: PIDGains) {
		p = gains.kP
		i = gains.kI
		d = gains.kD
		iZone = gains.kIZone
	}

	private fun setLeft(output: FractionalOutput) {
		if (isLeftAtOpenedLimit && output > 0.0) {
			leftMainMotor.set(0.0)
			return
		}
		if (isLeftAtClosedLimit && output < 0.0) {
			leftMainMotor.set(STAY_FOLDED_OUTPUT)
			return
		}
		leftMainMotor.set(output)
	}

	private fun setRight(output: FractionalOutput) {
		if (isRightAtOpenedLimit && output > 0.0) {
			rightMainMotor.set(0.0)
			return
		}
		if (isRightAtClosedLimit && output < 0.0) {
			rightMainMotor.set(STAY_FOLDED_OUTPUT)
			return
		}
		rightMainMotor.set(output)
	}

	override fun initSendable(builder: SendableBuilder) {
		super.initSendable(builder)
		builder.addBooleanProperty("Left at opened limit", { isLeftAtOpenedLimit }, null)
		builder.addBooleanProperty("Left at closed limit", { isLeftAtClosedLimit }, null)
		builder.addBooleanProperty("Right at opened limit", { isRightAtOpenedLimit }, null)
		builder.addBooleanProperty("Right at opened limit", { isRightAtClosedLimit }, null)
		builder.addDoubleProperty("Position rotations", { currentPosition }, null)
		builder.addDoubleProperty("Position setpoint rotations", { lastSetpoint }, null)
		builder.addBooleanProperty("In tolerance", { isWithinTolerance }, null)
	}

	override fun periodic() {
		if (isLeftAtClosedLimit) {
			leftEncoder.position = 0.0
		}
		if (isRightAtClosedLimit) {
			rightEncoder.position = 0.0
		}
	}
}
