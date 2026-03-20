package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ArmPosition extends Command {
    private RobotContainer m_robot;
    private int m_position;
    private double m_angle;

    public ArmPosition(RobotContainer robot, int position) {
        // Use addRequirements() here to declare subsystem dependencies
        // eg. addRequirements(chassis);
        addRequirements(robot.m_robotShooter);
        m_robot = robot;
        m_position = position;
        switch(position) {
            case 1 -> m_angle = 3.21;
            case 2 -> m_angle = 4.48;
            case 3 -> m_angle = 5.76;
        }
    }

    @Override
    public void execute() {
        m_robot.m_robotIntake.setAngle(m_angle);
        // m_robot.m_robotDrive.drive(3, 0, 0, true); // Drive forward/backward
    }

    @Override
    public boolean isFinished() {
        return false;
        // double currentAngle = m_robot.m_robotIntake.getEncoder();
        // return Math.abs(m_angle - currentAngle) < 0.1; // Finish when within 1 degree of target angle
    }

    @Override
    public void end(boolean interrupted) {
        m_robot.m_robotIntake.stopAll(); // Stop the robot
    }

}
