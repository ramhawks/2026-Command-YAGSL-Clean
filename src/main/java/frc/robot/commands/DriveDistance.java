package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveDistance extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private final double m_targetDistance;
    private Pose2d m_startPose;

    public DriveDistance(double meters, SwerveSubsystem subsystem) {
        m_swerveSubsystem = subsystem;
        m_targetDistance = meters;
        addRequirements(m_swerveSubsystem); // Prevents other commands from using the drivetrain
    }

    @Override
    public void initialize() {
        m_startPose = m_swerveSubsystem.getPose(); // Reset the encoders to start measuring distance from zero
    }

    @Override
    public void execute() {
        m_swerveSubsystem.drive(new ChassisSpeeds(1.0, 0.0, 0.0)); // Drive forward at 1 m/s (adjust as needed)
    }

    @Override
    public boolean isFinished() {
        // End the command once we've reached the target distance
        // Calculate current distance from the start point
        double currentDistance = m_swerveSubsystem
                                    .getPose()
                                    .getTranslation()
                                    .getDistance(m_startPose.getTranslation());
        return currentDistance >= m_targetDistance;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0)); // Stop the robot when the command ends
    }
}
