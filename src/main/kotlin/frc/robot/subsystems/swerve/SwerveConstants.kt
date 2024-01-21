package frc.robot.subsystems.swerve

import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig


object SwerveConstants {
	const val SWERVE_CONFIG_DIR = "swerve"
	
	/**meters per second*/
	const val MAX_SPEED = 4.2
	
	val PATH_CONSTRAINTS =
		PathConstraints(
			0.0, 0.0,
			0.0, 0.0
		)
	
	val PATH_TRANSLATION_CONSTANTS = PIDConstants(0.1, 0.0, 0.0)
	val PATH_ROTATION_CONSTANTS = PIDConstants(0.1, 0.0, 0.0)
	
	val PATH_PLANNER_CONFIG = HolonomicPathFollowerConfig(
		PIDConstants(0.1, 0.0, 0.0),
		PIDConstants(0.1, 0.0, 0.0),
		MAX_SPEED,
		0.417405,
		ReplanningConfig(),
	)
}