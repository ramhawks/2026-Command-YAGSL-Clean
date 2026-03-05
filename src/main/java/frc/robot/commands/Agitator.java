
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorRelay;

/**
 * Build out this Command if it is more complex than 3 to 5 lines of code or if will be re-used in multiple places.
 */
public class Agitator extends Command {
    private final AgitatorRelay m_agitator3000;

    /**
     * Creates a new Agitator command.
     *
     * @param agitatorRelay The subsystem used by this command.
     */
    public Agitator(AgitatorRelay agitatorRelay) {
        m_agitator3000 = agitatorRelay;
        
        // Add subsystem requirements here if needed.
        addRequirements(m_agitator3000);
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