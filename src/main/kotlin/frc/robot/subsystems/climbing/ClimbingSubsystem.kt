package frc.robot.subsystems.climbing

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.climbing.ClimbingConstants.ClimbingState
import frc.robot.RobotMap.Climbing as ClimbingMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem : SubsystemBase() {
    // --- Motors ---

    private val leftMainMotor = HaTalonFX(ClimbingMap.LEFT_MAIN_MOTOR_ID)
        .apply { configMainMotor() }

    private val leftSecondaryMotor = HaTalonFX(ClimbingMap.LEFT_SECONDARY_MOTOR_ID)
        .apply { configSecondaryMotor(ClimbingMap.LEFT_MAIN_MOTOR_ID) }

    private val rightMainMotor = HaTalonFX(ClimbingMap.RIGHT_MAIN_MOTOR_ID)
        .apply { configMainMotor() }

    private val rightSecondaryMotor = HaTalonFX(ClimbingMap.RIGHT_SECONDARY_MOTOR_ID)
        .apply { configSecondaryMotor(ClimbingMap.RIGHT_MAIN_MOTOR_ID) }


    // --- Motors Configuration ---

    private fun HaTalonFX.configMainMotor() =
        apply {
            // TODO: Verify positive output raises climber
            inverted = false
            configPIDGains(Constants.PID_GAINS)
            configurator.apply(Constants.FALCON_HARDWARE_LIMITS_CONFIG)
            configurator.apply(Constants.MOTION_MAGIC_CONFIG)
        }

    private fun HaTalonFX.configSecondaryMotor(mainMotorID: Int) =
        apply {
            // TODO: Check if follower motor should oppose master motor or not
            setControl(Follower(mainMotorID, false))
        }


    // --- Motors Properties ---

    var neutralMode = NeutralModeValue.Brake
        set(value) {
            leftMainMotor.setNeutralMode(value)
            leftSecondaryMotor.setNeutralMode(value)
            rightMainMotor.setNeutralMode(value)
            rightSecondaryMotor.setNeutralMode(value)
            field = value
        }

    private fun setMaxVelocity(maxVelocity: AngularVelocity) {
        val motionMagicConfig = Constants.MOTION_MAGIC_CONFIG.apply {
            MotionMagicCruiseVelocity = maxVelocity.rps
        }
        leftMainMotor.configurator.apply(motionMagicConfig)
        leftSecondaryMotor.configurator.apply(motionMagicConfig)
    }

    val withinTolerance
        get() = (leftMainMotor.closedLoopError.value <= Constants.SETPOINT_TOLERANCE) &&
                (rightMainMotor.closedLoopError.value <= Constants.SETPOINT_TOLERANCE)


    // --- Motors Control ---

    fun setClimbingStateSetpoint(state: ClimbingState) {
        setMaxVelocity(state.maxVelocity)

        val control = PositionVoltage(state.setpoint, 0.0, false, state.voltageFF, 0, false, false, false)
        leftMainMotor.setControl(control)
        rightMainMotor.setControl(control)
    }

    fun setSpeed(percentOutput: Double) {
        leftMainMotor.set(percentOutput)
        rightMainMotor.set(percentOutput)
    }
}