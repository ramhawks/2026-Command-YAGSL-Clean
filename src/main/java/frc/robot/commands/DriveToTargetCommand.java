// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * DriveToTargetCommand
 *
 * Drives the robot straight forward (robot-relative) at a fixed speed until
 * the Limelight detects an AprilTag on the target. Once a valid target is seen
 * (getTV() == true), the command ends and the robot stops.
 *
 * Point the robot roughly toward the target before running this command.
 * Designed to be followed by AimAtHubCommand for fine angular alignment.
 *
 * Always apply an external timeout (.withTimeout()) when using this in a
 * sequence so the robot does not drive forever if the tag is never seen.
 *
 * Typical usage:
 *
 *   Commands.sequence(
 *       new DriveToTargetCommand(swerve).withTimeout(10.0),
 *       new AimAtHubCommand(swerve),
 *       shootSequence
 *   );
 */
public class DriveToTargetCommand extends Command {

    private final SwerveSubsystem swerve;

    // Forward drive speed in meters per second.
    // Keep conservative (~0.75–1.0 m/s) so the robot has time to stop before
    // it overruns the target. Increase only after testing.
    private static final double DRIVE_SPEED_MPS = 0.75;

    public DriveToTargetCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        System.out.println("[DriveToTarget] Starting — driving toward target");
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
        SmartDashboard.putBoolean("DriveToTarget/HasTarget", hasTarget);

        if (!hasTarget) {
            // No tag yet — keep driving forward (robot-relative vx)
            swerve.setChassisSpeeds(new ChassisSpeeds(DRIVE_SPEED_MPS, 0.0, 0.0));
        } else {
            // Tag acquired — stop immediately
            swerve.setChassisSpeeds(new ChassisSpeeds());
        }
    }

    @Override
    public boolean isFinished() {
        // End as soon as the Limelight sees any valid target
        return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(new ChassisSpeeds());

        if (interrupted) {
            System.out.println("[DriveToTarget] Interrupted — no target found within timeout");
        } else {
            System.out.println("[DriveToTarget] Target detected — stopping drive");
        }
    }
}
