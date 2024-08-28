package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.function.DoubleSupplier;

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
        this.swerveDriveSubsystem.driveCommand(forwardAxis, strafeAxis, rotationAxis);
    }
}
