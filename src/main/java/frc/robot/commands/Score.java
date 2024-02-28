package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.ShooterSubsystem;

public class Score extends Command {

    private final ShooterSubsystem shoot;

    public Score(ShooterSubsystem shoot) {

        this.shoot = shoot;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}
