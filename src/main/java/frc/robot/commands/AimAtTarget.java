package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrainState;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.drive.Drive;

public class AimAtTarget extends Command {
    Drive driveSubsystem;
    Navigation navSubsystem;
    int targetId;

    public AimAtTarget(Drive driveSubsystem, Navigation navSubsystem, int targetId) {
        this.driveSubsystem = driveSubsystem;
        this.navSubsystem = navSubsystem;
        this.targetId = targetId;

        addRequirements(driveSubsystem, navSubsystem);
    }

    @Override
    public void initialize() {
        navSubsystem.setDesiredTarget(targetId);
        driveSubsystem.setDriveState(DriveTrainState.AIM);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setDriveState(DriveTrainState.MANUAL);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(navSubsystem.getCurrentHeading() - navSubsystem.getTargetHeading()) < 1e-5;
    }
}
