package frc.robot.commands

import com.hamosad1657.lib.commands.*
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Robot
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveSubsystem as Swerve

fun Swerve.pathfindThenFollowPathCommand(pathname: String): Command =
	AutoBuilder.pathfindThenFollowPath(
		PathPlannerPath.fromPathFile(pathname),
		SwerveConstants.PATH_CONSTRAINTS,
	)

fun Swerve.pathfindToInitialPoseThenFollowPathCommand(pathName: String): Command {
	val path = PathPlannerPath.fromPathFile(pathName)
	if (Robot.alliance == Alliance.Red) path.flipPath()

	val initialPose = path.pathPoses.first()
	return pathfindToPoseCommand(initialPose) andThen followPathCommand(path)
}

fun Swerve.pathfindToPoseCommand(pose: Pose2d): Command =
	AutoBuilder.pathfindToPose(pose, SwerveConstants.PATH_CONSTRAINTS)

fun Swerve.followAutoCommand(autoName: String): Command =
	AutoBuilder.buildAuto(autoName)

fun Swerve.followPathCommand(path: PathPlannerPath): Command =
	AutoBuilder.followPath(path)

fun Swerve.followPathCommand(pathName: String): Command =
	AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))
