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
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final MAXSwerveModule m_frontLeft =
        new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight =
        new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft =
        new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight =
        new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    // Nav-x2 Gyro
    public final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    public final PowerDistribution m_pdh = new PowerDistribution(9, PowerDistribution.ModuleType.kRev);

    // Field map
    // try {
    //     AprilTagFieldLayout m_FieldLayout = AprilTagFieldLayout.loadFromResource("/FRC2026_WELDED.fmap");
    // } catch (Exception e) {
    //     DriverStation.reportError(getName(), null);
    // }
    private final Field2d m_field = new Field2d();

    SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(m_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            });

    SwerveDrivePoseEstimator m_PoseEstimator = 
        new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
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
        // This method will be called once per scheduler run
        // Update the odometry in the periodic block
        // m_odometry.update(
        //     Rotation2d.fromDegrees(m_gyro.getAngle()), // Rotation2d.fromDegrees(-m_gyro.getAngle())
        //     new SwerveModulePosition[] {
        //         m_frontLeft.getPosition(),
        //         m_frontRight.getPosition(),
        //         m_rearLeft.getPosition(),
        //         m_rearRight.getPosition()
        //     }
        // );

        m_PoseEstimator.update(
            Rotation2d.fromDegrees(-m_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            }
        );

        // m_field.setRobotPose(m_odometry.getPoseMeters());
        m_field.setRobotPose(getPose());
        SmartDashboard.putData(m_field);
        SmartDashboard.putData(m_gyro);
        SmartDashboard.putData(m_pdh);

        // Log encoder positions to help with physical alignment
        // SmartDashboard.putNumber("FL Abs Encoder", m_frontLeft.getState().angle.getDegrees());
        // SmartDashboard.putNumber("FR Abs Encoder", m_frontRight.getState().angle.getDegrees());
        // SmartDashboard.putNumber("RL Abs Encoder", m_rearLeft.getState().angle.getDegrees());
        // SmartDashboard.putNumber("RR Abs Encoder", m_rearRight.getState().angle.getDegrees());
        // SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
    }
    
    public Pose2d getPose() {
        // return m_odometry.getPoseMeters();
        return m_PoseEstimator.getEstimatedPosition();
    }

    public double getVelocity() {
        double velocity = Math.sqrt(Math.pow(m_gyro.getVelocityX(), 2) + Math.pow(m_gyro.getVelocityY(), 2));
        return velocity;
    }

    public double getAccel() {
        double accel = Math.sqrt(Math.pow(m_gyro.getRawAccelX(), 2) + Math.pow(m_gyro.getRawAccelY(), 2));
        return accel;
    }

    /**
     * Resets the odometry to the specified pose.
     *.
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        // m_odometry.resetPosition(
        //     Rotation2d.fromDegrees(m_gyro.getAngle()),
        //     new SwerveModulePosition[] {
        //         m_frontLeft.getPosition(),
        //         m_frontRight.getPosition(),
        //         m_rearLeft.getPosition(),
        //         m_rearRight.getPosition()
        //     }, pose
        // );
        m_PoseEstimator.resetPosition(
            m_gyro.getRotation2d(), // Rotation2d.fromDegrees(m_gyro.getAngle())
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            }, pose
        );
    }

    public void addVision(Pose2d visionPose) {
        m_PoseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        // Get the current robot-relative speeds from the drivetrain
        ChassisSpeeds robotRelativeSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        );
        return robotRelativeSpeeds;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {

        double xSpeed = speeds.vxMetersPerSecond;
        double ySpeed = speeds.vyMetersPerSecond;
        double rot = speeds.omegaRadiansPerSecond;
        
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        boolean fieldRelative = false; // Since this method is for robot-relative speeds, we set fieldRelative to false

        var swerveModuleStates =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Sets the wheels into an X formation to prevent movement. */
    public Command setXCommand() {
        return this.run(
            () -> {
            m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
            m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
            }
        );
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public Command zeroHeadingCommand() {
        return this.runOnce(() -> m_gyro.reset());
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees(); // negative or not
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] m_states = new SwerveModuleState[4]; // SwerveModuleState[] m_states = new SwerveModuleState[3]
        m_states[0] = m_frontLeft.getState();
        m_states[1] = m_frontRight.getState();
        m_states[2] = m_rearLeft.getState();
        m_states[3] = m_rearRight.getState();
        return m_states;
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
