package frc.robot.subsystems.climbing

import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.math.PIDGains
import com.hamosad1657.lib.subsystemutils.setNameToClassName
import com.hamosad1657.lib.units.Rotations
import com.hamosad1657.lib.units.toIdleMode
import com.revrobotics.CANSparkBase.ControlType
import com.revrobotics.CANSparkBase.SoftLimitDirection
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel.MotorType
import com.revrobotics.SparkLimitSwitch
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Climbing as ClimbingMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem : SubsystemBase() {

	// --- Motors ---

	private val leftMainMotor = CANSparkFlex(ClimbingMap.LEFT_MAIN_MOTOR_ID, MotorType.kBrushless)
		.apply { configMainMotor() }

	private val leftSecondaryMotor = CANSparkFlex(ClimbingMap.LEFT_SECONDARY_MOTOR_ID, MotorType.kBrushless)
		.apply { configSecondaryMotor(leftMainMotor) }

	private val rightMainMotor = CANSparkFlex(ClimbingMap.RIGHT_MAIN_MOTOR_ID, MotorType.kBrushless)
		.apply { configMainMotor() }

	private val rightSecondaryMotor = CANSparkFlex(ClimbingMap.RIGHT_SECONDARY_MOTOR_ID, MotorType.kBrushless)
		.apply { configSecondaryMotor(rightMainMotor) }

	private val leftPIDController = leftMainMotor.pidController
	private val rightPIDController = rightMainMotor.pidController

	private var lastSetpoint: Rotations = 0.0

	// --- Motors Configuration ---

	private fun CANSparkFlex.configMainMotor() =
		apply {
			// TODO: Verify positive output raises climber
			inverted = false
			configPIDF(Constants.WEIGHT_BEARING_PID_GAINS)
			setSoftLimit(SoftLimitDirection.kForward, Constants.MAX_POSSIBLE_POSITION.toFloat())
			setSoftLimit(SoftLimitDirection.kReverse, Constants.MAX_POSSIBLE_POSITION.toFloat())
			enableSoftLimit(SoftLimitDirection.kForward, true)
			enableSoftLimit(SoftLimitDirection.kReverse, true)
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

	val isLeftAtOpenedLimit: Boolean
		get() = leftMainMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed

	val isLeftAtClosedLimit: Boolean
		get() = leftMainMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed

	val isRightAtOpenedLimit: Boolean
		get() = rightMainMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed

	val isRightAtClosedLimit: Boolean
		get() = rightMainMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed

	val isAtOpenedLimit: Boolean
		get() = isLeftAtOpenedLimit || isRightAtOpenedLimit

	val isAtClosedLimit: Boolean
		get() = isLeftAtClosedLimit || isRightAtClosedLimit


	// --- Motors Control ---

	fun configPIDF(gains: PIDGains) {
		leftMainMotor.configPIDF(gains)
		rightMainMotor.configPIDF(gains)
	}

	val currentPosition: Rotations
		get() = leftMainMotor.encoder.position

	fun setPositionSetpoint(newSetpoint: Rotations) {
		lastSetpoint = newSetpoint
		leftPIDController.setReference(newSetpoint, ControlType.kPosition)
		rightPIDController.setReference(newSetpoint, ControlType.kPosition)
	}

	fun increasePositionSetpointBy(desiredChangeInPosition: Rotations) {
		setPositionSetpoint(currentPosition + desiredChangeInPosition)
	}

	fun setPercentOutput(percentOutput: Double) {
		leftMainMotor.set(percentOutput)
		rightMainMotor.set(percentOutput)
	}

	private fun CANSparkFlex.configPIDF(gains: PIDGains) {
		pidController.p = gains.kP
		pidController.i = gains.kI
		pidController.d = gains.kD
		pidController.iZone = gains.kIZone
		pidController.ff = gains.kFF()
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
}