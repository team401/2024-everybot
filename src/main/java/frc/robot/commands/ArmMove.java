package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrainState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;

public class ArmMove extends Command {
    private final Arm armSub;
    private final Drive driveSub;
    private final boolean upDown;

    public ArmMove(Drive drive, Arm armSub, boolean upDown) {
        this.armSub = armSub;
        driveSub = drive;
        this.upDown = upDown;
    }

    @Override
    public void initialize() {
        armSub.armControl(upDown);
        driveSub.setDriveState(DriveTrainState.ENDGAME);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}
