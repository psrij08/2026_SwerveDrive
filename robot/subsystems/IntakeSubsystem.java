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
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax armMotor;
    private final SparkMax rollerMotor;
    private final AbsoluteEncoder armEncoder;

    private double Kp = DriveConstants.intakeArmKp;
    private double Ki = DriveConstants.intakeArmKi;
    private double Kd = DriveConstants.intakeArmKd;
    
    /** Creates a new DriveSubsystem. */
    public IntakeSubsystem() {
        armMotor = new SparkMax(DriveConstants.kIntakeArmMotorCanId, MotorType.kBrushed);
        rollerMotor = new SparkMax(DriveConstants.kIntakeRollerMotorCanId, MotorType.kBrushed);
        armEncoder = armMotor.getAbsoluteEncoder();
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
    public void startIntake(double speed) {
        rollerMotor.set(speed);
    }

    public void stopIntake() {
        rollerMotor.stopMotor();
    }

    public void stopAll() {
        armMotor.stopMotor();
        rollerMotor.stopMotor();
    }

    public void setAngle(double target){
        // armMotor.set(-1.0);
        // double currentTime = Timer.getFPGATimestamp();
        // while ((Timer.getFPGATimestamp() - currentTime) < 2.0){
        //     continue;
        // }
        // armMotor.set(0.0);

        double lastError = 0.0;
        double sumOfErrors = 0.0;
        double error = 1.0;
        double P, I, D, C;

        while (Math.abs(error) > 0.1){
            error = armEncoder.getPosition() - target;
            P = Kp * error;
            I = Ki * sumOfErrors;
            D = Kd * (error - lastError);
            C = 0.6 * Math.signum(error);

            armMotor.set(P + I + D + C);
            lastError = error;
            sumOfErrors = sumOfErrors + error;
            SmartDashboard.putNumber("arm motor set", P + I + D + C);
        }
    }

    public void moveArm(double speed) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double speedDelivered = speed * DriveConstants.kMaxIntakeSpeed;
        double position = armEncoder.getPosition();
        if (position < 3.21 || position > 5.76){
            armMotor.stopMotor();
        } else {
            if (position > 3.21 && position < 4.48 && Math.signum(speedDelivered) == -1.0){
                armMotor.set(speedDelivered);
            } else if (position > 4.48 && position < 5.76 && Math.signum(speedDelivered) == 1.0){
                armMotor.set(speedDelivered * 0.8);
            } else {
                armMotor.set(speedDelivered * 0.3);
            }
        }
    }

    public void position1() {
        armMotor.set(1);
    }

    public void position2() {
        if (armEncoder.getPosition() < 1.3) {
            armMotor.set(-1);
        } else if (armEncoder.getPosition() > 1.3) {
            armMotor.set(1);
        }
        // else {
        //     armMotor.stopMotor();
        // }
    }

    public void position3() {
        if (armEncoder.getPosition() > 2.6) {
            armMotor.set(1);
        } else {
            armMotor.set(-1);
        }
    }

    public double getEncoder() {
        return armEncoder.getPosition();
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        
    }
}
