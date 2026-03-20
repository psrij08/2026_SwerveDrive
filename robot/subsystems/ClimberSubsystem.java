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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor = new SparkMax(DriveConstants.kClimberMotorCanId, MotorType.kBrushed);
    private final DigitalInput limitTop = new DigitalInput(0);
    private final DigitalInput limitBottom = new DigitalInput(1);

    /** Creates a new DriveSubsystem. */
    public ClimberSubsystem() {
        // Throwable test = new Throwable();
        // DriverStation.reportError("drive constructor", test.getStackTrace());
        // RobotConfig config;
        // try {
        //     DriverStation.reportError("config start", test.getStackTrace());
        //     config = RobotConfig.fromGUISettings();
        //     // Configure AutoBuilder last
        //     AutoBuilder.configure(
        //         this::getPose, // Robot pose supplier
        //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        //             new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //             new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        //         ),
        //         config, // The robot configuration
        //         () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //             var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //         },
        //         this // Reference to this subsystem to set requirements
        //     );
        // } catch (Exception e) {
        //     // Handle exception as needed
        //     DriverStation.reportError("Small oops: " + e.getMessage(), e.getStackTrace());
        // }
    }

    @Override
    public void periodic() {

        // Log encoder positions to help with physical alignment
        // SmartDashboard.putNumber("FL Abs Encoder", m_frontLeft.getState().angle.getDegrees());
        // SmartDashboard.putNumber("FR Abs Encoder", m_frontRight.getState().angle.getDegrees());
        // SmartDashboard.putNumber("RL Abs Encoder", m_rearLeft.getState().angle.getDegrees());
        // SmartDashboard.putNumber("RR Abs Encoder", m_rearRight.getState().angle.getDegrees());
        // SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void climb(double speed) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double speedDelivered = speed * DriveConstants.kMaxClimberSpeed;
        if (limitTop.get() == true || limitBottom.get() == true){
            climberMotor.stopMotor();
        } else {
            climberMotor.set(speedDelivered);
        }
    }
}
