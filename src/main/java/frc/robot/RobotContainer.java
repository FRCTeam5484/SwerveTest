package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem;
    private final SwerveAuto autoCmd;
    private final XboxController m_joy0 = new XboxController(0);

    public RobotContainer() {
        swerveSubsystem = new SwerveSubsystem();
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_joy0));
        autoCmd = new SwerveAuto(swerveSubsystem);
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        
    }

    public SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    public Command getAutonomousCommand() {
        return autoCmd;
    }
}
