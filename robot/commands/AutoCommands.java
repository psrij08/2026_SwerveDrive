// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommands extends SequentialCommandGroup {
  private final double m_shootSpeed = 0.7;

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
          new Pose2d(3.75, 0.63, new Rotation2d(0)),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(2.75, 0.63), new Translation2d(1.75, 0.63)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(0.63, 0.63, new Rotation2d(180)),
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
          new Pose2d(0.63, 0.63, new Rotation2d(180)), // assumes rot2d = 0, possibly change to gyro
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(new Translation2d(1.22, 1.59), new Translation2d(1.82, 2.54)),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(2.41, 3.5, new Rotation2d(0)),
          config);

  private ProfiledPIDController thetaController =
      new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  /** Creates a new AutoCommands. */
  public AutoCommands(RobotContainer t_robotC) {
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addCommands(
      new SwerveControllerCommand(
          autoTrajectory3,
          t_robotC.m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          t_robotC.m_robotDrive::setModuleStates,
          t_robotC.m_robotDrive
      )
      // new WaitCommand(3),
      // new SwerveControllerCommand(
      //     autoTrajectory2,
      //     t_robotC.m_robotDrive::getPose, // Functional interface to feed supplier
      //     DriveConstants.kDriveKinematics,

      //     // Position controllers
      //     new PIDController(AutoConstants.kPXController, 0, 0),
      //     new PIDController(AutoConstants.kPYController, 0, 0),
      //     thetaController,
      //     t_robotC.m_robotDrive::setModuleStates,
      //     t_robotC.m_robotDrive
      // ),
      // new SpinUp(t_robotC, m_shootSpeed),
      // new WaitCommand(2),
      // new ShootCommand(t_robotC)
    );
  }
}
