package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class DriveWithGamepad extends Command {

    private SwerveDriveSubsystem swerveDriveSubsystem;

    private DoubleSupplier forwardAxis;
    private DoubleSupplier strafeAxis;
    private DoubleSupplier rotationAxis;

    public DriveWithGamepad(
            SwerveDriveSubsystem swerveDriveSubsystem,
            DoubleSupplier forwardAxis,
            DoubleSupplier strafeAxis,
            DoubleSupplier rotationAxis) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        this.forwardAxis = forwardAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;

        addRequirements(this.swerveDriveSubsystem);
    }

    @Override
    public void execute() {
        // Get the desired chassis speeds in ft/s, ft/s, and rad/s.
        double vx = this.forwardAxis.getAsDouble() * SwerveConstants.MAX_SPEED;
        double vy = this.strafeAxis.getAsDouble() * SwerveConstants.MAX_SPEED;
        double omega = this.rotationAxis.getAsDouble() * SwerveConstants.MAX_ANGULAR_VELOCITY;

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(vx, vy, omega);
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);

        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // Make the robot move
        swerveDriveSubsystem.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }
}
