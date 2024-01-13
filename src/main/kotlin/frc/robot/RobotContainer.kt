package frc.robot

import edu.wpi.first.wpilibj2.command.Command

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    init {
        configureBindings()
        setDefaultCommands()
    }

    /** Use this method to define your `trigger->command` mappings. */
    private fun configureBindings() {

    }

    private fun setDefaultCommands() {

    }

    fun getAutonomousCommand(): Command? {
        // TODO: Implement properly
        return null
    }
}