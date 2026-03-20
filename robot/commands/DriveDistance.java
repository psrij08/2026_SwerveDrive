package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DriveDistance extends Command {
    private RobotContainer m_robot;
    private double m_distance;

    public DriveDistance(RobotContainer robot, double distance) {
        // Use addRequirements() here to declare subsystem dependencies
        // eg. addRequirements(chassis);
        addRequirements(robot.m_robotDrive);
        m_robot = robot;
        m_distance = distance;
    }

    @Override
    public void initialize() {
        m_robot.m_robotDrive.resetOdometry(m_robot.m_robotDrive.getPose());
    }

    @Override
    public void execute() {
        double currentDistance = m_robot.m_robotDrive.getPose().getTranslation().getNorm();
        double error = m_distance - currentDistance;
        double speed = Math.signum(error) * Math.min(0.5, Math.abs(error)); // Simple proportional control
        m_robot.m_robotDrive.drive(speed, 0, 0, false); // Drive forward/backward
        // m_robot.m_robotDrive.drive(3, 0, 0, true); // Drive forward/backward
    }

    @Override
    public boolean isFinished() {
        double currentDistance = m_robot.m_robotDrive.getPose().getTranslation().getNorm();
        SmartDashboard.putNumber("Error", Math.abs(m_distance - currentDistance));
        return Math.abs(m_distance - currentDistance) < 0.03; // Finish when within 3 cm of target
    }

    @Override
    public void end(boolean interrupted) {
        m_robot.m_robotDrive.drive(0, 0, 0, true); // Stop the robot
    }

}
