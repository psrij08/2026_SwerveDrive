package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TurnAngle extends Command {
    private RobotContainer m_robot;
    private double m_angle;

    public TurnAngle(RobotContainer robot, double angle) {
        // Use addRequirements() here to declare subsystem dependencies
        // eg. addRequirements(chassis);
        addRequirements(robot.m_robotDrive);
        m_robot = robot;
        m_angle = angle;
    }

    @Override
    public void initialize() {
        m_robot.m_robotDrive.resetOdometry(m_robot.m_robotDrive.getPose());
    }

    @Override
    public void execute() {
        // double currentAngle = m_robot.m_robotDrive.getPose().getRotation().getDegrees();
        // double error = m_angle - currentAngle;
        // double speed = Math.signum(error) * Math.min(0.5, Math.abs(error)); // Simple proportional control
        double speed = 1.0;
        m_robot.m_robotDrive.drive(0, 0, speed, false); // Drive forward/backward
        // m_robot.m_robotDrive.drive(3, 0, 0, true); // Drive forward/backward
    }

    @Override
    public boolean isFinished() {
        double currentAngle = m_robot.m_robotDrive.getPose().getRotation().getDegrees();
        SmartDashboard.putNumber("Angular Error", Math.abs(m_angle - currentAngle));
        return Math.abs(m_angle - currentAngle) < 1; // Finish when within 1 degree of target ang;e
    }

    @Override
    public void end(boolean interrupted) {
        m_robot.m_robotDrive.drive(0, 0, 0, false); // Stop the robot
    }

}
