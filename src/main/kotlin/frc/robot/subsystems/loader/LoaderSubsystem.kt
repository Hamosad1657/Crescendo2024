package frc.robot.subsystems.loader

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap.Loader as LoaderMap

object LoaderSubsystem : SubsystemBase() {
    private val loaderMotor = WPI_TalonSRX(LoaderMap.MOTOR_ID)
    private val beamBreak = DigitalInput(LoaderMap.BEAM_BREAK_CHANNEL)

    var neutralMode = NeutralMode.Brake
        set(value) {
            loaderMotor.setNeutralMode(value)
            field = value
        }

    fun setLoader(percentOutput: Double) {
        loaderMotor.set(percentOutput)
    }

    // TODO: Check if beam-break sensor is wired normally-true or normally-false
    /** Beam-break is positioned between loader and shooter. */
    fun isNoteDetected() = beamBreak.get()
}