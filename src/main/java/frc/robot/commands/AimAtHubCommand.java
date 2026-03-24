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
 * AimAtHubCommand
 *
 * Rotates the robot in place until the Limelight's horizontal offset (tx)
 * to the AprilTag on the hub is within tolerance. Designed to run AFTER
 * PathPlanner has already driven the robot to the correct position — this
 * command handles the final fine-alignment step only.
 *
 * Uses a simple proportional (P) controller on tx. No translation is applied;
 * the robot only rotates.
 *
 * Usage in RobotContainer (after a driveToPose path):
 *
 *   Commands.sequence(
 *       m_swerveSubsystem.driveToPose(hubShootingPose),
 *       new AimAtHubCommand(m_swerveSubsystem)
 *   );
 *
 * Or bound to a button for manual use during teleop:
 *
 *   m_driverController.x().whileTrue(new AimAtHubCommand(m_swerveSubsystem));
 */
public class AimAtHubCommand extends Command {

    private final SwerveSubsystem swerve;

    // --- Tuning constants ---

    // How aggressively to rotate toward the target.
    // Increase if the robot is too slow to align; decrease if it oscillates.
    // Start at 0.04 and adjust in 0.01 increments.
    private static final double kP = 0.04;

    // Minimum rotation power to overcome static friction.
    // If the robot barely moves when tx is small, increase this slightly.
    private static final double kMinOutput = 0.02;

    // Maximum rotation power to prevent overshooting.
    private static final double kMaxOutput = 0.4;

    // How many degrees off-center counts as "aligned."
    // Tighter = more precise but may never finish. 1.5 deg is a good starting point.
    private static final double TOLERANCE_DEGREES = 1.5;

    // How many consecutive loops we must be within tolerance before finishing.
    // Prevents the command from ending on a single lucky frame.
    private static final int STABLE_LOOPS_REQUIRED = 5;

    private int stableLoopCount = 0;

    public AimAtHubCommand(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        stableLoopCount = 0;
        System.out.println("[AimAtHub] Starting alignment");
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);

        SmartDashboard.putBoolean("AimAtHub/HasTarget", hasTarget);

        if (!hasTarget) {
            // No tag visible — hold still and wait
            swerve.setChassisSpeeds(new ChassisSpeeds());
            stableLoopCount = 0;
            return;
        }

        // tx = horizontal angle from crosshair to target in degrees
        // Positive tx = target is to the RIGHT of center
        // Negative tx = target is to the LEFT of center
        double tx = LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);

        SmartDashboard.putNumber("AimAtHub/TX", tx);
        SmartDashboard.putNumber("AimAtHub/StableLoops", stableLoopCount);

        // Check if we're already within tolerance
        if (Math.abs(tx) < TOLERANCE_DEGREES) {
            stableLoopCount++;
            swerve.setChassisSpeeds(new ChassisSpeeds()); // Hold still
            return;
        }

        stableLoopCount = 0;

        // Proportional rotation output
        // Negative because: positive tx (target right) → rotate right (negative omega in WPILib)
        double rawOutput = -kP * tx;

        // Apply minimum output to overcome friction, preserving direction
        double omega;
        if (Math.abs(rawOutput) < kMinOutput) {
            omega = Math.copySign(kMinOutput, rawOutput);
        } else {
            omega = rawOutput;
        }

        // Clamp to max output
        omega = Math.max(-kMaxOutput, Math.min(kMaxOutput, omega));

        // Rotate only — no translation
        // Uses robot-relative ChassisSpeeds: vx=0, vy=0, omega=rotation
        swerve.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, omega));
    }

    @Override
    public boolean isFinished() {
        // Done once we've been stably centered for enough consecutive loops
        return stableLoopCount >= STABLE_LOOPS_REQUIRED;
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the robot when this command ends
        swerve.setChassisSpeeds(new ChassisSpeeds());

        if (interrupted) {
            System.out.println("[AimAtHub] Interrupted before alignment complete");
        } else {
            System.out.println("[AimAtHub] Alignment complete — tx within tolerance");
        }
    }
}