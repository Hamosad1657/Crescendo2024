package frc.robot.subsystems.arm

import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.units.FractionalOutput
import com.hamosad1657.lib.units.toIdleMode
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Arm as ArmMap
import frc.robot.subsystems.arm.ArmConstants as Constants

object ArmSubsystem : SubsystemBase() {

	private val leftMotor = CANSparkFlex(ArmMap.MOTOR_ID, MotorType.kBrushless)
	private val rightMotor = CANSparkFlex(ArmMap.MOTOR_ID, MotorType.kBrushless)

	init {
		leftMotor.inverted = false // TODO: Verify positive output opens arm.
		rightMotor.inverted = false
	}

	// TODO: Check if limit switches are wired normally open or normally closed.
	//  If normally open, add a ! to these getters.
	// Open circuit -> pin reads 5V -> DigitalInput.get() returns true
	// Closed circuit -> pin reads 0V -> DigitalInput.get() returns false
	val isLeftAtLimit: Boolean
		get() = leftMotor.outputCurrent >= Constants.CURRENT_AT_LIMIT
	val isRightAtLimit: Boolean
		get() = rightMotor.outputCurrent >= Constants.CURRENT_AT_LIMIT


	val isAtLimit get() = isLeftAtLimit || isRightAtLimit

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

	fun setBothMotors(output: FractionalOutput) {
		setLeftMotor(output)
		setRightMotor(output)
	}

	override fun initSendable(builder: SendableBuilder) {
		super.initSendable(builder)
		builder.addBooleanProperty("Is at limit", { isAtLimit }, null)
	}
}