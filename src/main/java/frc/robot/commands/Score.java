package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.Scoring;

public class Score extends Command {

    private final Scoring score;

    public Score(Scoring score) {

        this.score = score;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
}
