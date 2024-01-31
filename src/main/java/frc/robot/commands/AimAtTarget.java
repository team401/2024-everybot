package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    }

    @Override
    public void execute() {
        // navSubsystem.getTargetHeadingError();
        // navSubsystem.getCurrentHeading();
    }
}
