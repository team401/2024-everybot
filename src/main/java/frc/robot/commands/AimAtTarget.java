package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.drive.Drive;

public class AimAtTarget extends Command {
    Drive driveSubsystem;
    Navigation navSubsystem;

    public AimAtTarget(Drive driveSubsystem, Navigation navSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.navSubsystem = navSubsystem;

        addRequirements(driveSubsystem, navSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // send navSubsystem.aimAtTarget() -> Pose2d to drive subsystem
    }
}
