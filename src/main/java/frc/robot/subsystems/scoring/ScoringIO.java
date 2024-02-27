package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface ScoringIO {

    @AutoLog
    public static class ScoringIOInputs {
        public double leftShooterVolts = 0.0;
        public double leftShooterAppliedVolts = 0.0;
        public double leftShooterCurrentAmps = 0.0;
        public double leftShooterVelocityRPM = 0.0;

        public double rightShooterVolts = 0.0;
        public double rightShooterAppliedVolts = 0.0;
        public double rightShooterCurrentAmps = 0.0;
        public double rightShooterVelocityRPM = 0.0;

        public double kickerAppliedVolts = 0.0;
        public double kickerCurrentAmps = 0.0;
    }

    public default void updateInputs(ScoringIOInputs inputs) {}

    public default void setShooterVolts(double volts) {}

    public default void setKickerVolts(double volts) {}
}
