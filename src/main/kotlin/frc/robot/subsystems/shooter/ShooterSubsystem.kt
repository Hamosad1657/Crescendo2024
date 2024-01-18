package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.toIdleMode
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
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

    private val angleMotor = TalonFX(ShooterMap.Angle.MOTOR_ID).apply {
        // TODO: Verify positive output raises angle
        inverted = false

        FeedbackConfigs().apply {
            FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder
            FeedbackRemoteSensorID = ShooterMap.Angle.CANCODER_ID
        }.let { configurator.apply(it) }
    }

    private val angleCANCoder = CANcoder(ShooterMap.Angle.CANCODER_ID).apply {
        CANcoderConfiguration().apply {
            MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1
            MagnetSensor.MagnetOffset = Constants.CANCODER_OFFSET_DEG

            // TODO: Verify measurement gets more positive when going up after the minimum
            MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive
        }.let { configurator.apply(it) }
    }


    // --- Motors Configuration ---

    var shooterNeutralMode = NeutralMode.Coast
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
     * When the shooter is at it's lowest possible angle, measurement is 0.
     * When moving away the measurement gets more positive.
     */
    val angle get() = Rotation2d.fromRotations(angleMotor.position.value)

    // I have to track this myself because there is no getter for it in SparkPIDController :(
    private var velocitySetpoint = AngularVelocity.fromRpm(0.0)

    val withinTolerance get() = withinVelocityTolerance && withinAngleTolerance


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

    // --- Private Utility ---

    private val withinVelocityTolerance get() = velocity - velocitySetpoint <= Constants.VELOCITY_TOLERANCE

    private val withinAngleTolerance get() = angleMotor.closedLoopError.value <= Constants.ANGLE_TOLERANCE.rotations


    // --- Testing and Manual Overrides ---

    /** To be used in testing or in manual overrides. For normal operation use setShooterState. */
    fun setShooterMotorsOutput(percentOutput: Double) {
        shooterMainMotor.set(percentOutput)
    }

    /** To be used in testing or in manual overrides. For normal operation use setShooterState. */
    fun increaseShooterMotorsOutputBy(percentOutput: Double) {
        shooterMainMotor.set(shooterMainMotor.get() + percentOutput)
    }

    /** To be used in testing or in manual overrides. For normal operation use setShooterState. */
    fun increaseVelocitySetpointBy(velocity: AngularVelocity) {
        setVelocity(this.velocity + velocity)
    }

    /** To be used in testing or in manual overrides. For normal operation use setShooterState. */
    fun setAngleMotorOutput(percentOutput: Double) {
        angleMotor.set(percentOutput)
    }

    /** To be used in testing or in manual overrides. For normal operation use setShooterState. */
    fun increaseAngleSetpointBy(angle: Rotation2d) {
        setAngle(this.angle + angle)
    }
}