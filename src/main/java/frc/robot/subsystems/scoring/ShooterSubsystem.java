package frc.robot.subsystems.scoring;

import java.io.FileReader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.InterpolateDouble;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private InterpolateDouble rpmInterpolated;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        rpmInterpolated = new InterpolateDouble()
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
