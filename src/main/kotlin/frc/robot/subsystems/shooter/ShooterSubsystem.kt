package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.*
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.*
import com.revrobotics.CANSparkBase.IdleMode
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.shooter.ShooterConstants.ANGLE_MOTOR_TO_CANCODER_GEAR_RATIO
import frc.robot.subsystems.shooter.ShooterConstants.AngleMotorDirection
import frc.robot.subsystems.shooter.ShooterConstants.AngleMotorDirection.TOWARDS_MAX
import frc.robot.subsystems.shooter.ShooterConstants.AngleMotorDirection.TOWARDS_MIN
import frc.robot.subsystems.shooter.ShooterConstants.KEEP_AT_MAX_ANGLE_OUTPUT
import frc.robot.subsystems.shooter.ShooterConstants.KEEP_AT_MIN_ANGLE_OUTPUT
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_PID_GAINS
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState
import kotlin.math.absoluteValue
import frc.robot.RobotMap.Shooter as ShooterMap
import frc.robot.RobotMap.Shooter.Angle as ShooterAngleMap
import frc.robot.subsystems.shooter.ShooterConstants as Constants

object ShooterSubsystem : SubsystemBase() {

	// --- Motors and Sensors ---

	private val shooterMainMotor = HaSparkFlex(ShooterMap.UPPER_MOTOR_ID).apply {
		restoreFactoryDefaults()
		// TODO: Verify positive output shoots
		inverted = true
		idleMode = IdleMode.kCoast
	}

	private val shooterPIDController = SHOOTER_PID_GAINS.toPIDController()

	private val shooterSecondaryMotor = HaSparkFlex(ShooterMap.LOWER_MOTOR_ID).apply {
		restoreFactoryDefaults()
		idleMode = IdleMode.kCoast
		// TODO: Check if follower needs to be inverted from the main motor
		follow(shooterMainMotor, true)
	}

	private val shooterEncoder = shooterMainMotor.getEncoder()

	private val angleMotor = HaTalonFX(ShooterAngleMap.MOTOR_ID).apply {
		configurator.apply(TalonFXConfiguration())
		// TODO: Verify positive output raises angle
		inverted = false
		idleMode = IdleMode.kBrake

		configurator.apply(FeedbackConfigs().apply {
			FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
			FeedbackRemoteSensorID = ShooterAngleMap.CANCODER_ID
			RotorToSensorRatio = ANGLE_MOTOR_TO_CANCODER_GEAR_RATIO
		})
		configurator.apply(Slot0Configs().apply {
			kP = Constants.ANGLE_PID_GAINS.kP
			kI = Constants.ANGLE_PID_GAINS.kI
			kD = Constants.ANGLE_PID_GAINS.kD
		})
	}

	private val angleCANCoder = CANcoder(ShooterAngleMap.CANCODER_ID).apply {
		configurator.apply(
			CANcoderConfiguration().apply {
				MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1
				MagnetSensor.MagnetOffset = Constants.CANCODER_OFFSET.rotations

				// TODO: Verify measurement gets more positive when going up after the minimum
				MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
			}
		)
	}

	private val minAngleLimitSwitch = DigitalInput(ShooterAngleMap.MIN_ANGLE_LIMIT_CHANNEL)
	private val maxAngleLimitSwitch = DigitalInput(ShooterAngleMap.MAX_ANGLE_LIMIT_CHANNEL)


	// --- Motors Properties ---

	val currentVelocity get() = AngularVelocity.fromRpm(-shooterEncoder.velocity)

	/**
	 * When the shooter is at its lowest possible angle, measurement is 1 degree.
	 * When moving away the measurement gets more positive.
	 *
	 * - This value does not come directly from the CANCoder, but from the motor controller,
	 * which uses the CANCoder for feedback. Essentially, this is the position that the motor
	 * controller thinks it's at.
	 */
	val currentAngle: Rotation2d get() = Rotation2d.fromRotations(angleMotor.position.value)

	// I have to track this myself because there is no getter for it in SparkPIDController :(
	var currentVelocitySetpoint = AngularVelocity.fromRpm(0.0)
		private set

	// TODO: Verify this really is in rotations and not degrees.
	val currentAngleSetpoint: Rotation2d get() = Rotation2d.fromRotations(angleMotor.closedLoopReference.value)


	// --- Motors Control ---

	fun setShooterState(shooterState: ShooterState) {
		setAngle(shooterState.angle)
		setVelocity(shooterState.velocity)
	}

	fun setVelocity(velocitySetpoint: AngularVelocity) {
		currentVelocitySetpoint = velocitySetpoint
		val ff: Volts = SHOOTER_PID_GAINS.kFF(velocitySetpoint.asRpm)
		val pidOutput: Volts = shooterPIDController.calculate(currentVelocity.asRpm, velocitySetpoint.asRpm)
		shooterMainMotor.setVoltage(-(pidOutput + ff))
	}

	private val controlRequestShooterAngle = PositionVoltage(0.0).apply { EnableFOC = false }

	fun setAngle(angleSetpoint: Rotation2d) {
		when (angleMotorDirectionTo(angleSetpoint)) {
			TOWARDS_MIN -> if (isAtMinAngleLimit) return angleMotor.set(KEEP_AT_MIN_ANGLE_OUTPUT)
			TOWARDS_MAX -> if (isAtMaxAngleLimit) return angleMotor.set(KEEP_AT_MAX_ANGLE_OUTPUT)
		}

		angleMotor.setControl(controlRequestShooterAngle.apply {
			Position = angleSetpoint.rotations
			FeedForward = Constants.calculateAngleFF(currentAngle)
		})
	}


	private fun angleMotorDirectionTo(setpoint: Rotation2d): AngleMotorDirection =
		if (setpoint.rotations - currentAngle.rotations > 0.0) TOWARDS_MAX else TOWARDS_MIN

	fun stopShooterMotors() {
		shooterMainMotor.stopMotor()
	}

	fun stopAngleMotor() {
		angleMotor.stopMotor()
	}


	// --- Getters ---

	val isAtMinAngleLimit get() = minAngleLimitSwitch.get()
	val isAtMaxAngleLimit get() = maxAngleLimitSwitch.get()

	val isWithinVelocityTolerance get() = (currentVelocitySetpoint - currentVelocity).abs() <= Constants.VELOCITY_TOLERANCE
	val isWithinAngleTolerance get() = angleMotor.closedLoopError.value.absoluteValue <= Constants.ANGLE_TOLERANCE.rotations
	val isWithinTolerance get() = isWithinVelocityTolerance && isWithinAngleTolerance


	// --- Testing and Manual Overrides ---

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun setShooterMotorsOutput(output: PercentOutput) {
		shooterMainMotor.set(output)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun increaseShooterMotorsOutputBy(output: PercentOutput) {
		shooterMainMotor.set(shooterMainMotor.get() + output)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun increaseVelocitySetpointBy(velocity: AngularVelocity) {
		setVelocity(this.currentVelocity + velocity)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun setAngleMotorOutput(output: PercentOutput) {
		if (output > 0.0 && isAtMaxAngleLimit) {
			angleMotor.set(KEEP_AT_MAX_ANGLE_OUTPUT)
			return
		}
		if (output < 0.0 && isAtMinAngleLimit) {
			angleMotor.set(KEEP_AT_MIN_ANGLE_OUTPUT)
			return
		}
		angleMotor.set(output)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun increaseAngleSetpointBy(angle: Rotation2d) {
		setAngle(this.currentAngle + angle)
	}


	override fun initSendable(builder: SendableBuilder) {
		super.initSendable(builder)
		builder.addBooleanProperty("Is at min angle limit", { isAtMinAngleLimit }, null)
		builder.addBooleanProperty("Is at max angle limit", { isAtMaxAngleLimit }, null)
		builder.addDoubleProperty("Angle deg", { currentAngle.degrees }, null)
		builder.addDoubleProperty("CANCoder angle deg", { angleCANCoder.absolutePosition.value * 360 }, null)
		builder.addDoubleProperty("Angle setpoint deg", { currentAngleSetpoint.degrees }, null)
		builder.addDoubleProperty("Angle error deg", { angleMotor.closedLoopError.value * 360 }, null)
		builder.addBooleanProperty("Angle in tolerance", { isWithinAngleTolerance }, null)
		builder.addDoubleProperty("Velocity rpm", { currentVelocity.asRpm }, null)
		builder.addDoubleProperty("Velocity setpoint rpm", { currentVelocitySetpoint.asRpm }, null)
		builder.addDoubleProperty("Velocity error rpm", { (currentVelocitySetpoint - currentVelocity).asRpm }, null)
		builder.addBooleanProperty("Velocity in tolerance", { isWithinVelocityTolerance }, null)
		builder.addDoubleProperty("Angle motor output", { angleMotor.get() }, null)
		builder.addDoubleProperty("Shooter motors output", { shooterMainMotor.get() }, null)
	}
}
