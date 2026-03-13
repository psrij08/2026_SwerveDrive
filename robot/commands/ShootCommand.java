package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ShootCommand extends Command {
    private RobotContainer m_robot;

    public ShootCommand(RobotContainer robot) {
        // Use addRequirements() here to declare subsystem dependencies
        // eg. addRequirements(chassis);
        addRequirements(robot.m_robotShooter);
        m_robot = robot;
    }

    @Override
    public void execute() {
        m_robot.m_robotShooter.shoot(); // Start top motor at provided speed
        // m_robot.m_robotDrive.drive(3, 0, 0, true); // Drive forward/backward
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_robot.m_robotShooter.stopShooting(); // Stop the robot
    }

}
