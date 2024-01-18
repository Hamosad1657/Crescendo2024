package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.rotations
import com.hamosad1657.lib.units.rpm
import com.hamosad1657.lib.units.toIdleMode
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTING_ANGLE_TOLERANCE
import frc.robot.subsystems.shooter.ShooterConstants.SHOOTING_VELOCITY_TOLERANCE
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState

object ShooterSubsystem : SubsystemBase() {
    private val shooterMotor1 = CANSparkFlex(RobotMap.Shooter.SHOOTER_MOTOR_1_ID, MotorType.kBrushless)
    private val shooterMotor2 = CANSparkFlex(RobotMap.Shooter.SHOOTER_MOTOR_2_ID, MotorType.kBrushless).apply {
        this.follow(shooterMotor1, true) // TODO: Check if follower needs to be inverted from the main motor
    }
    private val shooterEncoder = shooterMotor1.getEncoder()

    // I have to track this myself because there is no getter for it in SparkPIDController :(
    private var velocitySetpoint = AngularVelocity.fromRpm(0.0)


    // TODO: Change to whatever motor controller we will use
    private val angleMotor = TalonFX(RobotMap.Shooter.ANGLE_MOTOR_ID)

    // ----------- Motor behaviour -----------

    var shooterNeutralMode = NeutralMode.Coast
        set(value) {
            shooterMotor1.idleMode = value.toIdleMode()
            shooterMotor2.idleMode = value.toIdleMode()
            field = value
        }

    var angleNeutralMode = NeutralModeValue.Brake
        set(value) {
            angleMotor.setNeutralMode(value)
            field = value
        }

    // ----------- Motor configuration -----------

    init {
        // TODO: Verify positive output shoots
        shooterMotor1.inverted = false

        // TODO: Verify positive output raises angle
        angleMotor.inverted = false
    }

    // ----------- Motor control -----------
    fun setShooterState(shooterState: ShooterState) {
        setAngle(shooterState.angle)
        setVelocity(shooterState.velocity)
    }

    // ----------- Getters -----------

    fun withinTolerance(): Boolean {
        return withinVelocityTolerance() && withinAngleTolerance()
    }

    fun getAngle() = angleMotor.position.value.rotations

    fun getVelocity() = shooterEncoder.velocity.rpm

    // ----------- Private utility methods -----------

    private fun setVelocity(velocity: AngularVelocity) {
        shooterMotor1.pidController.setReference(velocity.rpm, CANSparkBase.ControlType.kSmartVelocity)
        velocitySetpoint = velocity
    }

    private fun setAngle(angle: Rotation2d) {
        angleMotor.setControl(PositionVoltage(angle.rotations))
    }

    private fun getShooterError(): AngularVelocity {
        val velocity = shooterEncoder.velocity.rpm
        return velocity - velocitySetpoint
    }

    private fun withinVelocityTolerance(): Boolean {
        return getShooterError() <= SHOOTING_VELOCITY_TOLERANCE
    }

    private fun withinAngleTolerance(): Boolean {
        return angleMotor.closedLoopError.value <= SHOOTING_ANGLE_TOLERANCE.rotations
    }

    // ----------- For testing or manual overrides -----------

    /**
     * To be used in testing or in manual overrides.
     */
    fun setShooterMotorsOutput(percentOutput: Double) {
        shooterMotor1.set(percentOutput)
    }

    /**
     * To be used in testing or in manual overrides.
     */
    fun increaseVelocitySetpointBy(velocity: AngularVelocity) {
        setVelocity(getVelocity() + velocity)
    }

    /**
     * To be used in testing or in manual overrides.
     */
    fun setAngleMotorOutput(percentOutput: Double) {
        angleMotor.set(percentOutput)
    }

    /**
     * To be used in testing or in manual overrides.
     */
    fun increaseAngleSetpointBy(angle: Rotation2d) {
        setAngle(getAngle() + angle)
    }
}