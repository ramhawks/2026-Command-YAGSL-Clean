// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.AutoBotHub;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.File;
import java.util.concurrent.atomic.AtomicReference;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final SwerveSubsystem m_swerveSubsystem;
  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();
  
  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(DriverConstants.kDriverControllerPort);

  // The operator's controller
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser;

  private double speedScale = 1.0; // Default speed scale (100%)

  // Telemetry table for drive-related state
  private final NetworkTable driveTelemetryTable = NetworkTableInstance.getDefault().getTable("Telemetry").getSubTable("Drive");
  private final NetworkTableEntry speedScaleEntry = driveTelemetryTable.getEntry("SpeedScale");
  private final NetworkTableEntry slowModeEntry   = driveTelemetryTable.getEntry("SlowMode");

  // Config table for OnRedSide
  private final NetworkTableEntry onRedSideEntry = NetworkTableInstance.getDefault().getTable("Config").getEntry("OnRedSide");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize the swerve subsystem with the deploy directory
    m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    // Configure the trigger bindings
    configureBindings();

    // Register all named commands for use in auto paths. This allows us to use the AutoBuilder to create autonomous commands from PathPlanner paths that can call these named commands.
    registerNamedCommands();

    // Set the default value for the "OnRedSide" config entry if it hasn't been set yet. This allows you to use this entry in autonomous commands to adjust behavior based on which side of the field we're on.
    if (Double.isNaN(onRedSideEntry.getDouble(Double.NaN))) {
      onRedSideEntry.setBoolean(true); // Default to true if not set
    }

    // Build an auto chooser. This will use BlueTopAuto-Simple as the default option.
    autoChooser = AutoBuilder.buildAutoChooser("BlueTopAuto-Simple");

    // Publish chooser so Elastic can pick it up (shows under /SmartDashboard)
    SmartDashboard.putData("Auto choices", autoChooser);

    // Use addOption when you want to add non-PathPlanner autonomous commands to the chooser.
    // Add buildTestCommand(): Drive foward 2 meters, turn 360 degrees, then shoot for 3 seconds.
    autoChooser.addOption("Test: Forward 2m + turn 360 + shoot 3s", buildTestCommand());

    // Write initial values to the Network Tables.
    speedScaleEntry.setDouble(speedScale);
    slowModeEntry.setBoolean(false);
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
    // *********  OPERATOR CONTROLS  *********
    // LEFT BUMPER: INTAKE
    // While the left bumper on operator controller is held, intake Fuel at the default speed. When the button is released, stop.
    m_operatorController.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    
    // RIGHT BUMPER: SHOOT    
    // While the right bumper on the operator controller is held, spin up for 1 second, then launch fuel. When the button is released, stop.
    m_operatorController.rightBumper().whileTrue(
          Commands.sequence(
            // Spin-up the feeder and launcher
            ballSubsystem.spinUpCommand(),
            // This sequence pauses until launcherAtSpeed() returns true. If it doesn’t become true within 1.5 seconds, the wait step ends anyway (timeout).
            Commands.waitUntil(ballSubsystem::launcherAtSpeed).withTimeout(1.5),
            // This command has no timeout, it keeps running as long as the bumper is held (or until interrupted by something else that requires the same subsystem).
            ballSubsystem.launchCommand()
          )
    );      

    // A BUTTON: REVERSE INTAKE (EJECT)
    // While the A button is held on the operator controller, eject fuel back out the intake
    m_operatorController.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    // ************************************

    // *********  DRIVE CONTROLS  *********
    // START BUTTON: ZERO GYRO
    // Press the start button on the driver controller to zero the gyro on the swerve drive. This is important for field-oriented control.
    m_driverController.start()
        .onTrue(m_swerveSubsystem.runOnce(() -> m_swerveSubsystem.zeroGyro()));

    // A BUTTON: TOGGLE SPEED SCALE (slow mode)
    // Press the A button on the driver controller to toggle between full speed and half speed. Write the values to the network tables.
    m_driverController.a()
        .onTrue(Commands.runOnce(() -> {
          speedScale = 0.5;
          speedScaleEntry.setDouble(speedScale);  // write the speed scale to the network tables
          slowModeEntry.setBoolean(true);
        }))
        .onFalse(Commands.runOnce(() -> {
          speedScale = 1.0;
          speedScaleEntry.setDouble(speedScale);  // write the speed scale to the network tables
          slowModeEntry.setBoolean(false);
        }));

    // DRIVE COMMANDS
    // TESTING
    // Robot-centric drive command for testing and tuning the swerve drive.
    // Not field-oriented. Controls are relative to the robot's current orientation. 
    // Drive the robot with the left stick for translation and the right stick for rotation. Useful for testing and tuning the swerve drive
    Command driveFieldOrientedAngVel = m_swerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), 0.15));

    // COMPETITION
    // Drive the robot with the left stick for translation and the right stick for rotation. This is field-oriented, so the
    // controls are relative to the field rather than the robot's current orientation. This is generally more intuitive for drivers,
    // but may require more tuning to get right.

    // BASE 
    // Drive command with speed scaling applied to translation, but not rotation.
    /* Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) * speedScale,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) * speedScale,
        // Add speedScale to the rotation as well (right stick). Useful to reduce the rotation speed for better control at lower speeds. 
        () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.15),   // * speedScale
        () -> -MathUtil.applyDeadband(m_driverController.getRightY(), 0.15)); // * speedScale */

    // SQUARED
    // Drive command with speed scaling applied to translation, but not rotation and with squared joystick input shaping.
    // Shape axis can shape as square or cube. 
    Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveCommand(
        () -> -shapeAxis(MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), false) * speedScale,
        () -> -shapeAxis(MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), false) * speedScale,
        // Add speedScale to the rotation as well (right stick). Useful to reduce the rotation speed for better control at lower speeds. 
        () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.15),   // * speedScale
        () -> -MathUtil.applyDeadband(m_driverController.getRightY(), 0.15)); // * speedScale

    // EXPO
    // Drive command with speed scaling applied to translation, but not rotation and with exponential joystick input shaping.
    /* Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveCommand(
        () -> -expo(MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), DriverConstants.TRANSLATION_EXPO) * speedScale,
        () -> -expo(MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), DriverConstants.TRANSLATION_EXPO) * speedScale,
        // Add speedScale to the rotation as well (right stick). Useful to reduce the rotation speed for better control at lower speeds. 
        () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.15),   // * speedScale
        () -> -MathUtil.applyDeadband(m_driverController.getRightY(), 0.15)); // * speedScale */

    m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngVel);
    //m_swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public String getSelectedAutoName() {
    Command selected = autoChooser.getSelected();
    return (selected != null) ? selected.getName() : "None";
  }

  // Square or Cube the input while preserving the sign, to give finer control at low speeds. Joystick input shaping.
  // value is expected in [-1, 1]
  private static double shapeAxis(double value, boolean useCube) {
    return (useCube) ? Math.copySign(value * value * value, value) : Math.copySign(value * value, value);
  }

  // Exponential shaping of the input, with a tunable exponent. Joystick input shaping.
  private static double expo(double x, double expo) {
    // expo in [0..1]. 0 = linear, 1 = very soft near center
    return (1 - expo) * x + expo * x * x * x;
  }

  // Create a test command that drives forward 2 meters, turns 360 degrees, then shoots for 3 seconds. 
  // An example of how you to build a more complex command using your subsystems and commands. 
  // This command can be added to the auto chooser to run it in autonomous for testing.
  private Command buildTestCommand() {
    // 1) Drive forward 2 meters (robot-relative forward), using odometry distance
    Command driveForward2m = createDriveForwardMetersCommand(2.0);

    // 2) Turn 360 degrees (starter version): spin at fixed omega for the time needed
    double omegaRadPerSec = 1.5;  // tune: higher = faster spin
    double secondsFor360 = (2.0 * Math.PI) / omegaRadPerSec;

    Command turn360 = Commands.run(
        () -> m_swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, omegaRadPerSec)),
        m_swerveSubsystem
    ).withTimeout(secondsFor360)
     .finallyDo(() -> m_swerveSubsystem.setChassisSpeeds(new ChassisSpeeds()));

    // 3) Shoot for 3 seconds (spin up -> wait for atSpeed -> launch)
    Command shoot3s = Commands.sequence(
        ballSubsystem.spinUpCommand(),
        Commands.waitUntil(ballSubsystem::launcherAtSpeed).withTimeout(1.5),
        ballSubsystem.launchCommand().withTimeout(3.0)
    );

    // Put it all together
    Command testAuto = Commands.sequence(
        driveForward2m,
        turn360,
        shoot3s
    );

    return testAuto;
  }

  private Command createDriveForwardMetersCommand(double meters) {
    PIDController distancePid = new PIDController(2.0, 0.0, 0.0); // tune kP
    distancePid.setTolerance(0.05);                         // 5 cm

    AtomicReference<Pose2d> startPoseRef = new AtomicReference<>();

    return new FunctionalCommand(
        // init
        () -> {
          startPoseRef.set(m_swerveSubsystem.getPose());
          distancePid.reset();
        },
        // execute
        () -> {
          Pose2d start = startPoseRef.get();
          Pose2d current = m_swerveSubsystem.getPose();

          // delta pose in the start frame; X is "forward" distance traveled
          double traveled = current.relativeTo(start).getX();
          double output = distancePid.calculate(traveled, meters);

          // Clamp speed (m/s). Start conservative.
          double vx = MathUtil.clamp(output, -1.5, 1.5);

          m_swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(vx, 0.0, 0.0));
          distancePid.close();
        },
        // end
        (interrupted) -> m_swerveSubsystem.setChassisSpeeds(new ChassisSpeeds()),
        // isFinished
        () -> distancePid.atSetpoint(),
        // requirements
        m_swerveSubsystem
    );
  }

  private void registerNamedCommands() {
    // Register any commands that you want to be able to call from PathPlanner paths here. This allows you to use the AutoBuilder
    // to create autonomous commands from PathPlanner paths that can call these named commands.
    // PRO TIP: If the command grows beyond 3 to 5 lines or it will re-used, promote it to it's own class under commands

    // *********  AutoBotHub  *********
    // Command for shooting FUEL into the hub during autonomous.
    Command autoBotHub = Commands.sequence(
      ballSubsystem.spinUpCommand(),
      Commands.waitUntil(ballSubsystem::launcherAtSpeed).withTimeout(1.5),
      ballSubsystem.launchCommand().withTimeout(0.75)
    );
    
    NamedCommands.registerCommand("AutoBotHub", autoBotHub);
    // ************************************

    // **********  SomeCommand  *********
    // Command for...
    // Code goes here.
    // *********************************
  }
}
