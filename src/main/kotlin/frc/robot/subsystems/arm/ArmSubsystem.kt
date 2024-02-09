package frc.robot.subsystems.arm

import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.motors.HaCANSparkMax
import com.hamosad1657.lib.units.FractionalOutput
import com.hamosad1657.lib.units.toIdleMode
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.arm.ArmConstants.CURRENT_LIMIT_AMP
import frc.robot.RobotMap.Arm as ArmMap

object ArmSubsystem : SubsystemBase() {

	private val leftMotor = HaCANSparkMax(ArmMap.MOTOR_ID)
	private val rightMotor = HaCANSparkMax(ArmMap.MOTOR_ID)

	init {
		leftMotor.inverted = false // TODO: Verify positive output opens arm.
		rightMotor.inverted = false

		leftMotor.setSmartCurrentLimit(CURRENT_LIMIT_AMP)
		rightMotor.setSmartCurrentLimit(CURRENT_LIMIT_AMP)
	}

	private val leftForwardLimitSwitch = DigitalInput(ArmMap.LEFT_FORWARD_LIMIT_CHANNEL)
	private val leftReverseLimitSwitch = DigitalInput(ArmMap.LEFT_REVERSE_LIMIT_CHANNEL)
	private val rightForwardLimitSwitch = DigitalInput(ArmMap.RIGHT_FORWARD_LIMIT_CHANNEL)
	private val rightReverseLimitSwitch = DigitalInput(ArmMap.RIGHT_REVERSE_LIMIT_CHANNEL)

	// TODO: Check if limit switches are wired normally open or normally closed.
	//  If normally open, add a ! to these getters.
	// Open circuit -> pin reads 5V -> DigitalInput.get() returns true
	// Closed circuit -> pin reads 0V -> DigitalInput.get() returns false
	val isLeftAtForwardLimit get() = leftForwardLimitSwitch.get()
	val isLeftAtReverseLimit get() = leftReverseLimitSwitch.get()
	val isRightAtForwardLimit get() = rightForwardLimitSwitch.get()
	val isRightAtReverseLimit get() = rightReverseLimitSwitch.get()


	val isAtForwardLimit get() = isLeftAtForwardLimit || isRightAtForwardLimit
	val isAtReverseLimit get() = isLeftAtReverseLimit || isRightAtReverseLimit

	var neutralMode: NeutralModeValue = NeutralModeValue.Brake
		set(value) {
			leftMotor.setIdleMode(value.toIdleMode())
			rightMotor.setIdleMode(value.toIdleMode())
			field = value
		}

	fun setLeftMotor(output: FractionalOutput) {
		leftMotor.set(output)
	}

	fun setRightMotor(output: FractionalOutput) {
		rightMotor.set(output)
	}

	fun setLeftMotorWithLimits(output: FractionalOutput) {
		if (output < 0.0 && isLeftAtReverseLimit) {
			setLeftMotor(0.0)
			return
		}
		if (output > 0.0 && isLeftAtForwardLimit) {
			setLeftMotor(0.0)
			return
		}
		setLeftMotor(output)
	}

	fun setRightMotorWithLimits(output: FractionalOutput) {
		if (output < 0.0 && isRightAtReverseLimit) {
			setRightMotor(0.0)
			return
		}
		if (output > 0.0 && isRightAtForwardLimit) {
			setRightMotor(0.0)
			return
		}
		setRightMotor(output)
	}

	fun setBothMotors(output: FractionalOutput) {
		setLeftMotor(output)
		setRightMotor(output)
	}

	fun setBothMotorsWithLimits(output: FractionalOutput) {
		setLeftMotorWithLimits(output)
		setRightMotorWithLimits(output)
	}

	override fun initSendable(builder: SendableBuilder) {
		super.initSendable(builder)
		builder.addBooleanProperty("Left at forward limit", { isLeftAtForwardLimit }, null)
		builder.addBooleanProperty("Left at reverse limit", { isLeftAtReverseLimit }, null)
		builder.addBooleanProperty("Right at forward limit", { isRightAtForwardLimit }, null)
		builder.addBooleanProperty("Right at forward limit", { isRightAtForwardLimit }, null)
	}
}
