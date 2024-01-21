package frc.robot.subsystems.swerve

import com.hamosad1657.lib.units.meters
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig


object SwerveConstants {
	const val SWERVE_CONFIG_DIR = "swerve"

	/** Meters per Second */
	const val MAX_SPEED = 4.4

	val PATH_CONSTRAINTS =
		PathConstraints(
			0.0, 0.0,
			0.0, 0.0
		)

	val PATH_TRANSLATION_CONSTANTS = PIDConstants(0.1, 0.0, 0.0)
	val PATH_ROTATION_CONSTANTS = PIDConstants(0.1, 0.0, 0.0)

	private val DRIVEBASE_RADIUS = 0.417405.meters

	val PATH_PLANNER_CONFIG = HolonomicPathFollowerConfig(
		PIDConstants(0.1, 0.0, 0.0),
		PIDConstants(0.1, 0.0, 0.0),
		MAX_SPEED,
		DRIVEBASE_RADIUS.meters,
		ReplanningConfig(),
	)
}