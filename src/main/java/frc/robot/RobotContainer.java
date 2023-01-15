package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem;
    private final XboxController driverOne = new XboxController(0);

    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem();
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, driverOne));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        
    }

    public SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
