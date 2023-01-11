package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.cmdDrive_TeleOp;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubSystem = new SwerveSubsystem();
  private final XboxController driverOne = new XboxController(OIConstants.kDriverOneControllerPort);
  public RobotContainer() {
    swerveSubSystem.setDefaultCommand(new cmdDrive_TeleOp(
      swerveSubSystem, 
      () -> driverOne.getLeftX(), 
      () -> driverOne.getLeftY(), 
      () -> driverOne.getRightX(), 
      () -> true));
      
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
