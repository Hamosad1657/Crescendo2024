package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.hamosad1657.lib.motors.HaSparkFlex
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.PercentOutput
import com.hamosad1657.lib.units.Volts
import com.hamosad1657.lib.units.absoluteValue
import com.hamosad1657.lib.units.degrees
import com.hamosad1657.lib.units.rotations
import com.hamosad1657.lib.units.rpm
import com.revrobotics.CANSparkBase.ControlType
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
	// --- Motors & Sensors ---

	private val shooterMainMotor =
		HaSparkFlex(ShooterMap.UPPER_MOTOR_ID).apply {
			configShooterMotor(inverted = false)
		}

	private val shooterSecondaryMotor =
		HaSparkFlex(ShooterMap.LOWER_MOTOR_ID).apply {
			configShooterMotor(inverted = false)
			follow(shooterMainMotor, true)
		}

	private val angleMotor =
		HaTalonFX(ShooterAngleMap.MOTOR_ID).apply {
			restoreFactoryDefaults()
			inverted = false
			idleMode = IdleMode.kBrake

			configurator.apply(Constants.ANGLE_CURRENT_LIMITS)
			configurator.apply(Constants.ANGLE_MOTION_MAGIC_CONFIG)

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

			configurator.apply(ClosedLoopGeneralConfigs().apply {
				ContinuousWrap = false
			})
		}

	private val angleCANCoder =
		CANcoder(ShooterAngleMap.CANCODER_ID).apply {
			configurator.apply(CANcoderConfiguration().apply {
				MagnetSensor.MagnetOffset = Constants.CANCODER_OFFSET.rotations
				MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1
				MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
			})
		}

	private val minAngleLimitSwitch = DigitalInput(ShooterAngleMap.MIN_ANGLE_LIMIT_CHANNEL)
	private val maxAngleLimitSwitch = DigitalInput(ShooterAngleMap.MAX_ANGLE_LIMIT_CHANNEL)

	private val shooterPIDController = shooterMainMotor.pidController.apply {
		p = SHOOTER_PID_GAINS.kP
		i = SHOOTER_PID_GAINS.kI
		d = SHOOTER_PID_GAINS.kD
	}
	private val shooterEncoder = shooterMainMotor.encoder


	// --- Motors Configuration ---

	private fun HaSparkFlex.configShooterMotor(inverted: Boolean) {
		this.inverted = inverted
		idleMode = IdleMode.kCoast
		voltageNeutralDeadband = Constants.SHOOTER_VOLTAGE_NEUTRAL_DEADBAND
		closedLoopRampRate = Constants.SHOOTER_RAMP_RATE_SEC
	}

	var shooterIdleMode = IdleMode.kBrake
		set(value) {
			shooterMainMotor.idleMode = value
			shooterSecondaryMotor.idleMode = value
			field = value
		}

	var angleIdleMode = IdleMode.kBrake
		set(value) {
			angleMotor.idleMode = value
			field = value
		}


	// --- State Getters ---

	val currentVelocity get() = AngularVelocity.fromRpm(shooterEncoder.velocity)

	// I have to track this myself because there is no getter for it in SparkPIDController :(
	var currentVelocitySetpoint = AngularVelocity.fromRpm(0.0)
		private set

	/**
	 * When the shooter is at its lowest possible angle, measurement is 1 degree.
	 * When moving away the measurement gets more positive.
	 *
	 * - This value does not come directly from the CANCoder, but from the motor controller,
	 * which uses the CANCoder for feedback. Essentially, this is the position that the motor
	 * controller thinks it's at.
	 */
	val currentAngle: Rotation2d get() = Rotation2d.fromRotations(angleCANCoder.absolutePosition.value)

	val currentAngleSetpoint: Rotation2d get() = Rotation2d.fromRotations(angleMotor.closedLoopReference.value)

	val isAtMinAngleLimit get() = minAngleLimitSwitch.get()
	val isAtMaxAngleLimit get() = maxAngleLimitSwitch.get()

	val angleTolerance get() = Constants.ANGLE_TOLERANCE_TABLE.getOutputFor(DynamicShooting.getFlatDistanceToSpeaker()).degrees
	val velocityTolerance get() = Constants.VELOCITY_TOLERANCE_TABLE.getOutputFor(DynamicShooting.getFlatDistanceToSpeaker()).rpm

	val angleError get() = angleMotor.closedLoopError.value.rotations

	val isWithinVelocityTolerance
		get() = (currentVelocitySetpoint - currentVelocity).absoluteValue <= velocityTolerance
	val isWithinAngleTolerance get() = angleError.rotations.absoluteValue <= angleTolerance.degrees
	val isWithinTolerance get() = isWithinVelocityTolerance && isWithinAngleTolerance

	val isWithinAngleToleranceToAmp
		get() = (ShooterState.TO_AMP.angle - currentAngle).absoluteValue.rotations <= Constants.AMP_ANGLE_TOLERANCE.rotations

	private fun angleMotorDirectionTo(setpoint: Rotation2d): AngleMotorDirection =
		if (setpoint.rotations - currentAngle.rotations > 0.0) TOWARDS_MAX else TOWARDS_MIN


	// --- Motors Control ---

	fun resetVelocityPIDController() {
		shooterPIDController.setIAccum(0.0)
	}

	fun setShooterState(shooterState: ShooterState) {
		setVelocity(shooterState.velocity)
		setAngle(shooterState.angle)
	}

	fun setVelocity(velocitySetpoint: AngularVelocity) {
		currentVelocitySetpoint = velocitySetpoint

		// If the velocity setpoint is zero, don't actively try to get to zero, just coast.
		if (velocitySetpoint.asRpm == 0.0) {
			stopShooterMotors()
			return
		}

		val velocityDelta = (shooterEncoder.velocity.rpm - velocitySetpoint).absoluteValue
		if (velocityDelta < velocityTolerance) {
			resetVelocityPIDController()
		}

		val ff: Volts = SHOOTER_PID_GAINS.kFF(velocitySetpoint.asRpm)
		shooterPIDController.setReference(velocitySetpoint.asRpm, ControlType.kVelocity, 0, ff)
	}

	private val controlRequestShooterAngle = MotionMagicVoltage(0.0).apply { EnableFOC = false }

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

	fun setShooterMotorsVoltage(voltage: Volts) {
		shooterMainMotor.setVoltage(voltage)
	}

	fun stopShooterMotors() {
		shooterMainMotor.stopMotor()
	}

	fun stopAngleMotor() {
		angleMotor.stopMotor()
	}


	// --- Telemetry ---

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("Subsystem")
		builder.addStringProperty("Command", { currentCommand?.name ?: "none" }, null)
		builder.addBooleanProperty("Is at min angle limit", { isAtMinAngleLimit }, null)
		builder.addBooleanProperty("Is at max angle limit", { isAtMaxAngleLimit }, null)
		builder.addDoubleProperty("Angle deg", { currentAngle.degrees }, null)
		builder.addDoubleProperty("CANCoder angle deg", { angleCANCoder.absolutePosition.value * 360 }, null)
		builder.addDoubleProperty("Angle setpoint deg", { currentAngleSetpoint.degrees }, null)
		builder.addDoubleProperty("Angle error deg", { angleMotor.closedLoopError.value * 360 }, null)
		builder.addDoubleProperty("Angle tolerance deg", { angleTolerance.degrees }, null)
		builder.addBooleanProperty("Angle in tolerance", { isWithinAngleTolerance }, null)
		builder.addDoubleProperty("Velocity rpm", { currentVelocity.asRpm }, null)
		builder.addDoubleProperty("Velocity setpoint rpm", { currentVelocitySetpoint.asRpm }, null)
		builder.addDoubleProperty("Velocity error rpm", { (currentVelocitySetpoint - currentVelocity).asRpm }, null)
		builder.addDoubleProperty("velocity tolerance rpm", { velocityTolerance.asRpm }, null)
		builder.addBooleanProperty("Velocity in tolerance", { isWithinVelocityTolerance }, null)
		builder.addDoubleProperty("Angle motor output", { angleMotor.get() }, null)
		builder.addDoubleProperty("Shooter motors output", { shooterMainMotor.get() }, null)
	}


	// --- Testing & Manual Overrides ---

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun setShooterMotorsOutput(output: PercentOutput) {
		shooterMainMotor.set(output)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun increaseVelocitySetpointBy(velocity: AngularVelocity) {
		setVelocity(currentVelocity + velocity)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun setAngleMotorOutput(output: PercentOutput) {
		if (output > 0.0 && isAtMaxAngleLimit) {
			return angleMotor.set(KEEP_AT_MAX_ANGLE_OUTPUT)
		}
		if (output < 0.0 && isAtMinAngleLimit) {
			return angleMotor.set(KEEP_AT_MIN_ANGLE_OUTPUT)
		}
		angleMotor.set(output)
	}

	/** To be used in testing or in manual overrides. For normal operation use setShooterState. */
	fun increaseAngleSetpointBy(angle: Rotation2d) {
		setAngle(currentAngle + angle)
	}
}
