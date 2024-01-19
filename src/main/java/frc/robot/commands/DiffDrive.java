package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DiffDrive extends Command{


    private final Drive drivesub;
    private final DoubleSupplier xPercent;
    private final DoubleSupplier yPercent;

    public DiffDrive(Drive d, DoubleSupplier xPcent, DoubleSupplier yPcent) {
        drivesub = d;
        xPercent = xPcent;
        yPercent = yPcent;
  
        addRequirements(drivesub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drivesub.setDiffDriveControls(xPercent.getAsDouble(), yPercent.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
    }


}
