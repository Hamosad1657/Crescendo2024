package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.FractionalOutput
import com.hamosad1657.lib.units.toIdleMode
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.shooter.ShooterConstants.ANGLE_MOTOR_TO_CANCODER_GEAR_RATIO
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import frc.robot.RobotMap.Shooter as ShooterMap
import frc.robot.subsystems.shooter.ShooterConstants as Constants

object ShooterSubsystem : SubsystemBase() {

	// --- Motors and Sensors ---

	private val shooterMainMotor = CANSparkFlex(ShooterMap.MAIN_MOTOR_ID, MotorType.kBrushless).apply {
		// TODO: Verify positive output shoots
		inverted = false
	}

	private val shooterSecondaryMotor = CANSparkFlex(ShooterMap.SECONDARY_MOTOR_ID, MotorType.kBrushless).apply {
		follow(shooterMainMotor, true) // TODO: Check if follower needs to be inverted from the main motor
	}

	private val shooterEncoder = shooterMainMotor.getEncoder()

	private val angleMotor = HaTalonFX(ShooterMap.Angle.MOTOR_ID).apply {
		// TODO: Verify positive output raises angle
		inverted = false

		configurator.apply(
			FeedbackConfigs().apply {
				FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
				FeedbackRemoteSensorID = ShooterMap.Angle.CANCODER_ID
				RotorToSensorRatio = ANGLE_MOTOR_TO_CANCODER_GEAR_RATIO
			}
		)
		configurator.apply(Constants.FALCON_HARDWARE_LIMITS_CONFIG)
	}

	private val angleCANCoder = CANcoder(ShooterMap.Angle.CANCODER_ID).apply {
		CANcoderConfiguration().apply {
			MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1
			MagnetSensor.MagnetOffset = Constants.CANCODER_OFFSET.rotations

			// TODO: Verify measurement gets more positive when going up after the minimum
			MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
		}.let { configurator.apply(it) }
	}


	// --- Motors Configuration ---

	var shooterNeutralMode = NeutralModeValue.Coast
		set(value) {
			shooterMainMotor.idleMode = value.toIdleMode()
			shooterSecondaryMotor.idleMode = value.toIdleMode()
			field = value
		}

	var angleNeutralMode = NeutralModeValue.Brake
		set(value) {
			angleMotor.setNeutralMode(value)
			field = value
		}


	// --- Motors Properties ---

	val velocity get() = AngularVelocity.fromRpm(shooterEncoder.velocity)

	/**
	 * When the shooter is at its lowest possible angle, measurement is 1 degree.
	 * When moving away the measurement gets more positive.
	 *
	 * - This value does not come directly from the CANCoder, but from the motor controller,
	 * which uses the CANCoder for feedback. Essentially, this is the position that the motor
	 * controller thinks it's at.
	 */
	val angle get() = Rotation2d.fromRotations(angleMotor.position.value)

	// I have to track this myself because there is no getter for it in SparkPIDController :(
	var velocitySetpoint = AngularVelocity.fromRpm(0.0)
		private set

	// TODO: Verify this really is in rotations and not degrees.
	val angleSetpoint get() = Rotation2d.fromRotations(angleMotor.closedLoopReference.value)


	// --- Motor control ---

	fun setShooterState(shooterState: ShooterState) {
		setAngle(shooterState.angle)
		setVelocity(shooterState.velocity)
	}

	private fun setVelocity(velocity: AngularVelocity) {
		shooterMainMotor.pidController.setReference(velocity.rpm, CANSparkBase.ControlType.kSmartVelocity)
		velocitySetpoint = velocity
	}

	private fun setAngle(angle: Rotation2d) {
		angleMotor.setControl(PositionVoltage(angle.rotations))
	}

	// --- Getters ---

	// TODO: Check if switches are wired normally open or normally closed.
	//  If they are wired normally closed, replace the 0 with a 1 and delete this comment.

	val isAtMaxAngleLimit: Boolean
		get() = angleMotor.getForwardLimit().value.value == 0

	val isAtMinAngleLimit: Boolean
		get() = angleMotor.getReverseLimit().value.value == 0

	val isWithinVelocityTolerance get() = velocity - velocitySetpoint <= Constants.VELOCITY_TOLERANCE

	val isWithinAngleTolerance get() = angleMotor.closedLoopError.value <= Constants.ANGLE_TOLERANCE.rotations

	val isWithinTolerance get() = isWithinVelocityTolerance && isWithinAngleTolerance


	// --- Testing and Manual Overrides ---

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun setShooterMotorsOutput(output: FractionalOutput) {
		shooterMainMotor.set(output)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun increaseShooterMotorsOutputBy(output: FractionalOutput) {
		shooterMainMotor.set(shooterMainMotor.get() + output)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun increaseVelocitySetpointBy(velocity: AngularVelocity) {
		setVelocity(this.velocity + velocity)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun setAngleMotorOutput(output: FractionalOutput) {
		angleMotor.set(output)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun increaseAngleSetpointBy(angle: Rotation2d) {
		setAngle(this.angle + angle)
	}

	override fun initSendable(builder: SendableBuilder) {
		super.initSendable(builder)
		builder.addBooleanProperty("Is at min angle limit", { isAtMinAngleLimit }, null)
		builder.addBooleanProperty("Is at max angle limit", { isAtMaxAngleLimit }, null)
		builder.addDoubleProperty("Angle deg", { angle.degrees }, null)
		builder.addDoubleProperty("Angle setpoint deg", { angleSetpoint.degrees }, null)
		builder.addBooleanProperty("Angle in tolerance", { isWithinAngleTolerance }, null)
		builder.addDoubleProperty("Velocity rpm", { velocity.rpm }, null)
		builder.addDoubleProperty("Velocity setpoint rpm", { velocitySetpoint.rpm }, null)
		builder.addBooleanProperty("Velocity in tolerance", { isWithinVelocityTolerance }, null)
	}
}
