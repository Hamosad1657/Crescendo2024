package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkBase.IdleMode
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel.MotorType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ShooterSubsystem : SubsystemBase() {
	private val angleMotor =
		TalonSRX(RobotMap.Shooter.ANGLE_MOTOR_ID) // TODO: Change to whatever motor controller we will use

	private val shooterMotor1 = CANSparkFlex(RobotMap.Shooter.SHOOTER_MOTOR_1_ID, MotorType.kBrushless)
	private val shooterMotor2 = CANSparkFlex(RobotMap.Shooter.SHOOTER_MOTOR_2_ID, MotorType.kBrushless)

	var shooterIdleMode: IdleMode = IdleMode.kBrake
		get() = shooterMotor1.idleMode
		set(value) {
			shooterMotor1.idleMode = value
			shooterMotor2.idleMode = value
			field = value
		}

	var angleIdleMode: IdleMode = IdleMode.kBrake
		set(value) {
			when (value) {
				IdleMode.kBrake -> angleMotor.setNeutralMode(NeutralMode.Brake)
				IdleMode.kCoast -> angleMotor.setNeutralMode(NeutralMode.Coast)
			}
			field = value
		}
	

	init {
		angleMotor.inverted = false // TODO: Verify positive output raises angle
		angleIdleMode = IdleMode.kBrake

		// TODO: Verify positive output shoots
		shooterMotor1.inverted = false
		shooterMotor2.inverted = false
		shooterIdleMode = IdleMode.kCoast
	}
}