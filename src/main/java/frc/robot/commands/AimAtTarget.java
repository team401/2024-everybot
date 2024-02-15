package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlignState;
import frc.robot.Constants.DriveTrainState;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.drive.Drive;

public class AimAtTarget extends Command {
    Drive driveSubsystem;
    Navigation navSubsystem;
    AlignState alignTarget;

    public AimAtTarget(Drive driveSubsystem, Navigation navSubsystem, AlignState alignTarget) {
        this.driveSubsystem = driveSubsystem;
        this.navSubsystem = navSubsystem;
        this.alignTarget = alignTarget;

        addRequirements(driveSubsystem, navSubsystem);
    }

    private Alliance getAlliance() {
        boolean ally = DriverStation.getAlliance().isPresent();
        if (ally) {
            return DriverStation.getAlliance().get();
        }
        return Alliance.Red;
    }

    private int getTargetId() {
        switch (alignTarget) {
            case SPEAKER:
                return getAlliance() == Alliance.Blue ? 7 : 4;
            case AMP:
                return getAlliance() == Alliance.Blue ? 6 : 5;
            case ENDGAME:
                return getAlliance() == Alliance.Blue ? 15 : 12;
            default:
                return 0;
        }
    }

    @Override
    public void initialize() {
        navSubsystem.setDesiredTarget(getTargetId());
        driveSubsystem.setDriveState(DriveTrainState.AIM);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setDriveState(DriveTrainState.MANUAL);
    }
}
