package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scoring extends SubsystemBase {

    private final ScoringIO scoringIO;
    private final ScoringIOInputsAutoLogged scoringInputs = new ScoringIOInputsAutoLogged();

    public Scoring(ScoringIO score) {
        this.scoringIO = score;
    }

    @Override
    public void periodic() {}

    public void shoot() {
        scoringIO.setShooterVolts(0); // placeholder
    }
}
