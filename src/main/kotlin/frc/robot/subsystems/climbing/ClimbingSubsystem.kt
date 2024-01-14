package frc.robot.subsystems.climbing

import com.ctre.phoenix6.controls.Follower
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Climber as ClimberMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem : SubsystemBase() {
    private val rightMotor1 = HaTalonFX(ClimberMap.RIGHT_MOTOR_1_ID).apply {
        configPIDGains(Constants.PID_GAINS)
    }
    private val rightMotor2 = HaTalonFX(ClimberMap.RIGHT_MOTOR_2_ID).apply {
        // TODO: Check if follower motor should oppose master motor or not
        setControl(Follower(ClimberMap.RIGHT_MOTOR_1_ID, false))
    }

    private val leftMotor1 = HaTalonFX(ClimberMap.LEFT_MOTOR_1_ID).apply {
        configPIDGains(Constants.PID_GAINS)
    }
    private val leftMotor2 = HaTalonFX(ClimberMap.LEFT_MOTOR_2_ID).apply {
        // TODO: Check if follower motor should oppose master motor or not
        setControl(Follower(ClimberMap.LEFT_MOTOR_1_ID, false))
    }

    
}