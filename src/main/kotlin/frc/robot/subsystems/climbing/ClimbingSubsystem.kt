package frc.robot.subsystems.climbing

import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Climber as RobotMap

object ClimbingSubsystem : SubsystemBase() {
    private val firstRightMotor = HaTalonFX(RobotMap.FIRST_RIGHT_MOTOR_ID)
    private val secondRightMotor = HaTalonFX(RobotMap.SECOND_RIGHT_MOTOR_ID).apply {
        set
    }

    private val firstLeftMotor = HaTalonFX(RobotMap.FIRST_LEFT_MOTOR_ID)
    private val secondLeftMotor = HaTalonFX(RobotMap.SECOND_LEFT_MOTOR_ID)


}