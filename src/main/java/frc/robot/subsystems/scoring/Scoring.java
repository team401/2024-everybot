package frc.robot.subsystems.scoring;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Scoring extends SubsystemBase {

    private final ShooterIO scoringIO;
    private final ShooterIOInputsAutoLogged scoringInputs = new ShooterIOInputsAutoLogged();

    public Scoring(ShooterIO score) {
        this.scoringIO = score;
    }

    @Override
    public void periodic() {}

    public void shoot() {
        scoringIO.setShooterVolts(0); // placeholder
    }
}
