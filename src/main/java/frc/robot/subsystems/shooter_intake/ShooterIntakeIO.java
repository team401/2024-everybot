package frc.robot.subsystems.shooter_intake;

public interface ShooterIntakeIO {

    public static class ShooterIntakeIOInputs {
        public double intakeVoltage = 0.0;
    }

    public void setVoltage(double volts);
}
