package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.units.toIdleMode
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ShooterSubsystem : SubsystemBase() {
    private val shooterMotor1 = CANSparkFlex(RobotMap.Shooter.SHOOTER_MOTOR_1_ID, MotorType.kBrushless)
    private val shooterMotor2 = CANSparkFlex(RobotMap.Shooter.SHOOTER_MOTOR_2_ID, MotorType.kBrushless).apply {
        this.follow(
            shooterMotor1, true
        )
    }

    // TODO: Change to whatever motor controller we will use
    private val angleMotor = TalonFX(RobotMap.Shooter.ANGLE_MOTOR_ID)

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

    init {
        // TODO: Verify positive output shoots
        shooterMotor1.inverted = false
        shooterMotor2.inverted = false

        // TODO: Verify positive output raises angle
        angleMotor.inverted = false
    }
}