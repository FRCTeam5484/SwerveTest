package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final XboxController controller;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, XboxController controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println(
            swerveSubsystem.frontLeft.getAbsoluteEncoderRad() + " " +
            swerveSubsystem.frontRight.getAbsoluteEncoderRad() + " " +
            swerveSubsystem.backLeft.getAbsoluteEncoderRad() + " " +
            swerveSubsystem.backRight.getAbsoluteEncoderRad()
        );
        double turningSpeed = controller.getLeftX() * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        double xSpeed = (-controller.getLeftTriggerAxis() + controller.getRightTriggerAxis()) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0.0, turningSpeed);
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}