package frc.robot.subsystems.climbing

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Climber as ClimberMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem : SubsystemBase() {
    private val leftMotor1 = HaTalonFX(ClimberMap.LEFT_MOTOR_1_ID).apply {
        configPIDGains(Constants.PID_GAINS)
        configurator.apply(Constants.FALCON_HARDWARE_LIMITS_CONFIG)
        configurator.apply(Constants.MOTION_MAGIC_CONFIG)

    }
    private val leftMotor2 = HaTalonFX(ClimberMap.LEFT_MOTOR_2_ID).apply {
        // TODO: Check if follower motor should oppose master motor or not
        setControl(Follower(ClimberMap.LEFT_MOTOR_1_ID, false))
    }

    private val rightMotor1 = HaTalonFX(ClimberMap.RIGHT_MOTOR_1_ID).apply {
        configPIDGains(Constants.PID_GAINS)
        configurator.apply(Constants.FALCON_HARDWARE_LIMITS_CONFIG)
        configurator.apply(Constants.MOTION_MAGIC_CONFIG)
    }
    private val rightMotor2 = HaTalonFX(ClimberMap.RIGHT_MOTOR_2_ID).apply {
        // TODO: Check if follower motor should oppose master motor or not
        setControl(Follower(ClimberMap.RIGHT_MOTOR_1_ID, false))
    }


    var neutralMode = NeutralModeValue.Brake
        set(value) {
            leftMotor1.setNeutralMode(value)
            leftMotor2.setNeutralMode(value)
            rightMotor1.setNeutralMode(value)
            rightMotor2.setNeutralMode(value)
            field = value
        }

    fun setRotationSetpoint(rotations: Double, limitSpeed: Boolean) {
        if (limitSpeed) {
            leftMotor1.setControl(MotionMagicVoltage(rotations))
            rightMotor1.setControl(MotionMagicVoltage(rotations))
        } else {
            leftMotor1.setControl(PositionVoltage(rotations))
            rightMotor1.setControl(PositionVoltage(rotations))
        }
    }

}