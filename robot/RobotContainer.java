// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO: Finish this file

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final ShooterSubsystem m_robotShooter = new ShooterSubsystem();

    // private final SendableChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    // public final Joystick m_flightStick = new Joystick(OIConstants.kFlightStickPort);
    public final CommandJoystick m_flightStick = new CommandJoystick(OIConstants.kFlightStickPort);
    public final CommandXboxController m_secondaryController =
        new CommandXboxController(OIConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        // // Build an auto chooser. This will use Commands.none() as the default option.
        // // autoChooser = AutoBuilder.buildAutoChooser();
        // // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("Example Path");

        // SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure default commands
        m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () ->
                    m_robotDrive.drive(
                        -MathUtil.applyDeadband(
                            m_flightStick.getRawAxis(1), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(
                            m_flightStick.getRawAxis(0), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(
                            m_flightStick.getRawAxis(2), OIConstants.kDriveDeadband),
                        true), // true or false
                m_robotDrive
            )
        );
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
      // new Trigger(m_exampleSubsystem::exampleCondition)
      //     .onTrue(new ExampleCommand(m_exampleSubsystem));

      m_flightStick.button(5).whileTrue(new SpinUp(this, 0.6)); // 0.71
      m_flightStick.button(6).whileTrue(new ShootCommand(this));

      m_flightStick.button(7)
      .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    m_flightStick.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    m_flightStick.getRawAxis(0), OIConstants.kDriveDeadband),
                LimelightHelpers.getTX("limelight") * -0.05,
                true
            ),
            m_robotDrive
      ));

      // Left Stick Button -> Set swerve to X
      m_secondaryController.leftStick().whileTrue(m_robotDrive.setXCommand());

      // Start Button -> Zero swerve heading
      m_secondaryController.start().onTrue(m_robotDrive.zeroHeadingCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      // return Autos.exampleAuto(m_exampleSubsystem);

      // Create config for trajectory
      TrajectoryConfig config =
          new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

      // An example trajectory to follow. All units in meters.
      Trajectory autoTrajectory1 =
          TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)),
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(3, 0, new Rotation2d(0)),
              config);

      Trajectory autoTrajectory3 = 
        TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(3.75, 0.63, new Rotation2d(0)),
                new Pose2d(0.63, 0.63, new Rotation2d(0))
            ),
            config
        );

      Trajectory autoTrajectory2 = TrajectoryGenerator.generateTrajectory(
              // Start at the origin facing the +X direction
              new Pose2d(0, 0, new Rotation2d(0)), // assumes rot2d = 0, possibly change to gyro
              // Pass through these two interior waypoints, making an 's' curve path
              List.of(new Translation2d(0, 0.6), new Translation2d(0, 1.3)),
              // End 3 meters straight ahead of where we started, facing forward
              new Pose2d(0, 2, new Rotation2d(0)),
              config);

      Trajectory autoTrajectories = autoTrajectory1.concatenate(autoTrajectory2);

      var thetaController =
          new ProfiledPIDController(
              AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand =
          new SwerveControllerCommand(
              autoTrajectory1,
              m_robotDrive::getPose, // Functional interface to feed supplier
              DriveConstants.kDriveKinematics,

              // Position controllers
              new PIDController(AutoConstants.kPXController, 0, 0),
              new PIDController(AutoConstants.kPYController, 0, 0),
              thetaController,
              m_robotDrive::setModuleStates,
              m_robotDrive);

      // Reset odometry to the starting pose of the trajectory.
      // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

      

      // Run path following command, then stop at the end.
      return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }

    // Alternate for PathPlanner
    // public Command getAutonomousCommand() {
    //     // This method loads the auto when it is called, however, it is recommended
    //     // to first load your paths/autos when code starts, then return the
    //     // pre-loaded auto/path.

    //     try {
    //         // Load the path you want to follow using its name in the GUI
    //         PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    //         // Create a path following command using AutoBuilder. This will also trigger event markers.
    //         return AutoBuilder.followPath(path);
    //     } catch (Exception e) {
    //         DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //         return Commands.none();
    //     }
    // }
}
