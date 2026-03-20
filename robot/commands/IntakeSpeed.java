package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeSpeed extends Command {
    private RobotContainer m_robot;
    private double m_speed;

    public IntakeSpeed(RobotContainer robot, double speed) {
        // Use addRequirements() here to declare subsystem dependencies
        // eg. addRequirements(chassis);
        addRequirements(robot.m_robotIntake);
        m_robot = robot;
        m_speed = speed;
    }

    @Override
    public void execute() {
        if (m_speed == 0.0) {
            m_robot.m_robotIntake.stopIntake();
        } else {
            m_robot.m_robotIntake.startIntake(m_speed);
        }
        // m_robot.m_robotDrive.drive(3, 0, 0, true); // Drive forward/backward
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_robot.m_robotIntake.stopIntake(); // Stop the robot
    }

}
