package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;

/**
 * Build out this Command if it is more complex than 3 to 5 lines of code or if will be re-used in multiple places.
 */
public class AutoBotHub extends Command {
    private final CANFuelSubsystem m_subsystem;

    /**
     * Creates a new AutoBotHub command.
     *
     * @param canFuelSubsystem The subsystem used by this command.
     */
    public AutoBotHub(CANFuelSubsystem canFuelSubsystem) {
        m_subsystem = canFuelSubsystem;
        
        // Add subsystem requirements here if needed.
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        // Initialization logic.
    }

    @Override
    public void execute() {
        // Repeatedly called while the command is scheduled.
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup logic when the command ends or is interrupted.
    }

    @Override
    public boolean isFinished() {
        // Return true when the command should finish.
        return false;
    }
}