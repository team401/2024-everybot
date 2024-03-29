package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrainState;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class ArcadeDrive extends Command {

    private final Drive drivesub;
    private final DoubleSupplier xPercent;
    private final DoubleSupplier yPercent;

    public ArcadeDrive(Drive d, DoubleSupplier xPcent, DoubleSupplier yPcent) {
        drivesub = d;
        xPercent = xPcent;
        yPercent = yPcent;

        addRequirements(drivesub);
    }

    @Override
    public void initialize() {
        drivesub.setDriveState(DriveTrainState.MANUAL);
    }

    @Override
    public void execute() {
        drivesub.setArcadeDrive(xPercent.getAsDouble(), yPercent.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {}
}
