// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.*;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    Pose2d init_pose = new Pose2d();
    m_robotContainer.m_robotDrive.resetOdometry(init_pose);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // LimelightHelpers.SetRobotOrientation("limelight", m_robotContainer.m_robotDrive.getHeading(), kDefaultPeriod, kDefaultPeriod, kDefaultPeriod, kDefaultPeriod, kDefaultPeriod);

    double heading = m_robotContainer.m_robotDrive.getHeading();
    double omegaRps = Units.degreesToRotations(m_robotContainer.m_robotDrive.getTurnRate());
    SmartDashboard.putNumber("Heading", heading);
    SmartDashboard.putNumber("OmegaRPS", omegaRps);
    LimelightHelpers.SetRobotOrientation("limelight", heading, omegaRps, 0.0, 0.0, 0.0, 0.0);
    // LimelightHelpers.PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    if (mt2Estimate != null && mt2Estimate.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
      // m_robotContainer.m_robotDrive.resetOdometry(mt2Estimate.pose);
      m_robotContainer.m_robotDrive.addVision(mt2Estimate.pose);
    }

    // how many degrees back is your limelight rotated from perfectly vertical?-
    double limelightMountAngleDegrees = 26.4; // 25

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.5; 

    // distance from the target to the floor
    double goalHeightInches = 44.25; 

    //calculate distance
    double angleToGoalRadians = (limelightMountAngleDegrees + LimelightHelpers.getTY("limelight"))  * (3.14159 / 180.0);
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    double distanceHorizontal = distanceFromLimelightToGoalInches * Math.cos(angleToGoalRadians);

    SmartDashboard.putNumber("Distance to Target", distanceHorizontal);
    SmartDashboard.putNumber("Arm Angle", m_robotContainer.m_robotIntake.getEncoder());

    SmartDashboard.putData("Drive", m_robotContainer.m_robotDrive);
    SmartDashboard.putNumber("ACtual Speed (m/s)", m_robotContainer.m_robotDrive.getVelocity());
    SmartDashboard.putNumber("Acceleration", m_robotContainer.m_robotDrive.getAccel());
    SmartDashboard.putNumber("Omega (rot/s)", omegaRps);
    SmartDashboard.putNumberArray("X", new double[]{Math.round(m_robotContainer.m_robotDrive.getPose().getTranslation().getX() * 100) / 100.0, Math.round(m_robotContainer.m_robotDrive.getPose().getTranslation().getY() * 100) / 100.0});

    

    // LimelightHelpers.SetRobotOrientation("limelight", m_robotContainer.m_robotDrive.getHeading(), 0, 0, 0, 0, 0);
    // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // if (mt2 != null && mt2.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
    //   m_robotContainer.m_robotDrive.resetOdometry(mt2.pose);
    // }

    // boolean doRejectUpdate = false;
    
    // // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    // if(Math.abs(omegaRps) < 1.0){
    //   doRejectUpdate = true;
    // }
    // // if our vision measurement has no targets, ignore vision updates
    // if(mt2.tagCount == 0){
    //   doRejectUpdate = true;
    // }
    // // if our vision measurement is more than 1 second old, ignore vision updates
    // if(mt2.latency > 1.0){
    //   doRejectUpdate = true;
    // }
    // // if our vision measurement is more than 10 meters away, ignore vision updates
    // if(mt2.pose.getTranslation().getNorm() > 10.0){
    //   doRejectUpdate = true;
    // }
    // if(!doRejectUpdate){
    //   //m_robotContainer.m_robotDrive.m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    //   m_robotContainer.m_robotDrive.resetOdometry(mt2.pose);
    // }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    AutoCommands m_autonomousCommand = new AutoCommands(m_robotContainer);
    // m_autonomousCommand = new DriveDistance(m_robotContainer, 1);

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   CommandScheduler.getInstance().schedule(m_autonomousCommand);
    // }

    // CommandScheduler.getInstance().schedule(m_autonomousCommand);

    CommandScheduler.getInstance().schedule(m_autonomousCommand);

    Pose2d init_pose = new Pose2d();
    Pose2d autoA_pose = new Pose2d(3.75, 0.63, Rotation2d.fromDegrees(0));
    m_robotContainer.m_robotDrive.resetOdometry(autoA_pose);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
