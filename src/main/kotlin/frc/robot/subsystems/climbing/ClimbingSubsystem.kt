package frc.robot.subsystems.climbing

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.subsystemutils.setNameToClassName
import com.hamosad1657.lib.units.AngularVelocity
import com.hamosad1657.lib.units.Rotations
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.climbing.ClimbingConstants.ClimbingState
import frc.robot.RobotMap.Climbing as ClimbingMap
import frc.robot.subsystems.climbing.ClimbingConstants as Constants

object ClimbingSubsystem : SubsystemBase() {
	init {
		setNameToClassName()
	}

	// --- Motors ---

	private val leftMainMotor = HaTalonFX(ClimbingMap.LEFT_MAIN_MOTOR_ID)
		.apply { configMainMotor() }

	private val leftSecondaryMotor = HaTalonFX(ClimbingMap.LEFT_SECONDARY_MOTOR_ID)
		.apply { configSecondaryMotor(ClimbingMap.LEFT_MAIN_MOTOR_ID) }

	private val rightMainMotor = HaTalonFX(ClimbingMap.RIGHT_MAIN_MOTOR_ID)
		.apply { configMainMotor() }

	private val rightSecondaryMotor = HaTalonFX(ClimbingMap.RIGHT_SECONDARY_MOTOR_ID)
		.apply { configSecondaryMotor(ClimbingMap.RIGHT_MAIN_MOTOR_ID) }

	private var lastFFVolts = 0.0

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

	/** Sets max velocity when using closed loop. */
	fun setMaxVelocity(maxVelocity: AngularVelocity) {
		val motionMagicConfig = Constants.MOTION_MAGIC_CONFIG.apply {
			MotionMagicCruiseVelocity = maxVelocity.rps
		}
		leftMainMotor.configurator.apply(motionMagicConfig)
		rightMainMotor.configurator.apply(motionMagicConfig)
	}

	val isWithinTolerance
		get() = (leftMainMotor.closedLoopError.value <= Constants.SETPOINT_TOLERANCE) &&
				(rightMainMotor.closedLoopError.value <= Constants.SETPOINT_TOLERANCE)


	// --- Switches ---

	// TODO: Check if switches are wired normally open or normally closed.
	//  If they are wired normally closed, replace the 0 with a 1 and delete this comment.

	val isAtOpenedLimit: Boolean
		get() {
			val leftSideAtForwardLimit = (leftMainMotor.getForwardLimit().value.value == 0)
			val rightSideAtForwardLimit = (rightMainMotor.getForwardLimit().value.value == 0)
			return leftSideAtForwardLimit || rightSideAtForwardLimit
		}

	val isAtClosedLimit: Boolean
		get() {
			val leftSideAtReverseLimit = (leftMainMotor.getReverseLimit().value.value == 0)
			val rightSideAtReverseLimit = (rightMainMotor.getReverseLimit().value.value == 0)
			return leftSideAtReverseLimit || rightSideAtReverseLimit
		}


	// --- Motors Control ---

	/**
	 * Does not config the max velocity, because that is an expensive operation, and the
	 * [setClimbingStateSetpoint] function is meant to be called periodically (for motor safety).
	 *
	 * Set the max velocity separately using the [setMaxVelocity] function. This only needs to
	 * be called once (every time you change it).
	 */
	fun setClimbingStateSetpoint(state: ClimbingState) {
		this.lastFFVolts = state.voltageFF
		val control = PositionVoltage(state.setpoint, 0.0, false, lastFFVolts, 0, false, false, false)
		leftMainMotor.setControl(control)
		rightMainMotor.setControl(control)
	}

	fun increasePositionSetpointBy(changeInPosition: Rotations) {
		val currentPosition = leftMainMotor.position.value
		val newSetpoint = currentPosition + changeInPosition

		val control = PositionVoltage(newSetpoint, 0.0, false, lastFFVolts, 0, false, false, false)
		leftMainMotor.setControl(control)
		rightMainMotor.setControl(control)
	}

	fun setSpeed(percentOutput: Double) {
		leftMainMotor.set(percentOutput)
		rightMainMotor.set(percentOutput)
	}


	// --- Orchestra ---

	object Orchestra {
		private var songs: Array<String>? = null
		private var instruments: ArrayList<TalonFX> = arrayListOf(
			leftMainMotor,
			leftSecondaryMotor,
			rightMainMotor,
			rightSecondaryMotor
		)

		private val orchestra = com.ctre.phoenix6.Orchestra(instruments as Collection<ParentDevice>)

		fun loadSongOptions(fileNames: Array<String>) {
			songs = fileNames
		}

		fun selectAndLoadSong(index: Int) {
			require(songs != null) { "Song options not loaded. Call [loadSongOptions]." }
			orchestra.loadMusic(songs!![index])
		}

		fun playSelectedSong() {
			orchestra.play()
		}

		fun pause() {
			orchestra.pause()
		}

		fun reset() {
			orchestra.stop()
		}

		val isPlaying: Boolean
			get() = orchestra.isPlaying
	}
}