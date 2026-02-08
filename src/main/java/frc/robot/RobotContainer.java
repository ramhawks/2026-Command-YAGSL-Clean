// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem m_swerveSubsystem;
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize the swerve subsystem with the deploy directory
    m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed, cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // Intake controls (19)
    //m_driverController.a().whileTrue(m_intakeSubsystem.intakeCommand());
    //m_driverController.y().whileTrue(m_intakeSubsystem.reverseIntakeCommand());
    
    // Shooter controls (9)
    //m_driverController.x().whileTrue(m_shooterSubsystem.shootCommand());
    
    // Test Code
    // Map the A button
    // While held: spin at 50% speed. When released: stop.
    m_driverController.a()
        .whileTrue(m_intakeSubsystem.run(() -> m_intakeSubsystem.setSpeed(0.5)))
        .onFalse(m_intakeSubsystem.runOnce(() -> m_intakeSubsystem.stop()));
    
    // Map the B button
    // To go in reverse, just map another button (like B) to -0.5
    m_driverController.b()
        .whileTrue(m_intakeSubsystem.run(() -> m_intakeSubsystem.setSpeed(-0.5)))
        .onFalse(m_intakeSubsystem.runOnce(() -> m_intakeSubsystem.stop()));

    // Map the X button
    //m_driverController.x()
    //    .whileTrue(m_shooterSubsystem.run(() -> m_shooterSubsystem.setSpeed(0.5)))
    //    .onFalse(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.stop()));

    // Map the Y button
    //m_driverController.y()
    //    .whileTrue(m_shooterSubsystem.run(() -> m_shooterSubsystem.setSpeed(-0.5)))
    //    .onFalse(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.stop()));
    m_driverController.y().whileTrue(m_swerveSubsystem.run(() -> m_swerveSubsystem.centerModulesCommand()));
    
    // End Test code
    
    // Drive with left stick
    /* m_swerveSubsystem.setDefaultCommand(
        m_swerveSubsystem.driveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()
        )
    ); */

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightY());

    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
