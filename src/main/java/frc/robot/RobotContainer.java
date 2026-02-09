// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final ExampleSubsystem m_exampleSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  
  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(DriverConstants.kDriverControllerPort);

  // The operator's controller
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_exampleSubsystem = new ExampleSubsystem();
    // Initialize the swerve subsystem with the deploy directory
    m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    
    // Configure the trigger bindings
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    //autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(m_driverController, ballSubsystem));
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
    // While the left bumper on operator controller is held, intake Fuel
    //m_operatorController.leftBumper()
    //    .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    //m_operatorController.rightBumper()
    //    .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
    //        .andThen(ballSubsystem.launchCommand())
    //        .finallyDo(() -> ballSubsystem.stop()));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    //m_operatorController.a()
    //    .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). The X-axis is also inverted so a positive value (stick to the right)
    // results in clockwise rotation (front of the robot turning right). Both axes
    // are also scaled down so the rotation is more easily controllable.
    //m_driverController.setDefaultCommand(
    //    m_driverController.driveArcade(
    //        () -> -m_driverController.getLeftY() * DRIVE_SCALING,
    //        () -> -m_driverController.getRightX() * ROTATION_SCALING));
    
            

    

    
    
    // Test Code
    // Map the A button
    // While held: spin at 50% speed. When released: stop.
    //m_driverController.a()
    //    .whileTrue(m_intakeSubsystem.run(() -> m_intakeSubsystem.setSpeed(0.5)))
    //    .onFalse(m_intakeSubsystem.runOnce(() -> m_intakeSubsystem.stop()));
    
    // Map the B button
    // To go in reverse, just map another button (like B) to -0.5
    //m_driverController.b()
    //    .whileTrue(m_intakeSubsystem.run(() -> m_intakeSubsystem.setSpeed(-0.5)))
    //    .onFalse(m_intakeSubsystem.runOnce(() -> m_intakeSubsystem.stop()));

    // Map the X button
    //m_driverController.x()
    //    .whileTrue(m_shooterSubsystem.run(() -> m_shooterSubsystem.setSpeed(0.5)))
    //    .onFalse(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.stop()));

    // Map the Y button
    //m_driverController.y()
    //    .whileTrue(m_shooterSubsystem.run(() -> m_shooterSubsystem.setSpeed(-0.5)))
    //    .onFalse(m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.stop()));
    //m_driverController.y().whileTrue(m_swerveSubsystem.run(() -> m_swerveSubsystem.centerModulesCommand()));
    
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
