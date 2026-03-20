// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax topMotor;
    private final SparkMax bottomMotor;
    private final SparkFlex feederMotor;
    
    /** Creates a new DriveSubsystem. */
    public ShooterSubsystem() {
        topMotor = new SparkMax(DriveConstants.kShooterTopMotorCanId, MotorType.kBrushed);
        bottomMotor = new SparkMax(DriveConstants.kShooterBottomMotorCanId, MotorType.kBrushed);
        feederMotor = new SparkFlex(DriveConstants.kFeederMotorCanId, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Update the odometry in the periodic block
        
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @return 
     */
    public void startShooter(double speed) {
        topMotor.set(speed);
    }

    public void shoot() {
        bottomMotor.set(1.0);
        feederMotor.set(-0.4);
    }

    public void stopShooter() {
        topMotor.stopMotor();
    }

    public void stopShooting() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
        feederMotor.stopMotor();
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        
    }
}
