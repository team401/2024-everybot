package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface ScoringIO {

    @AutoLog
    public static class ScoringIOInputs{ 
        public double shooterLeftVelocityRPM = 0.0;
        public double shooterLeftGoalVelocityRPM = 0.0;
        public double shooterLeftAppliedVolts = 0.0;
        public double shooterLeftCurrentAmps = 0.0;

        public double shooterRightVelocityRPM = 0.0;
        public double shooterRightGoalVelocityRPM = 0.0;
        public double shooterRightAppliedVolts = 0.0;
        public double shooterRightCurrentAmps = 0.0;

        public double kickerAppliedVolts = 0.0;
        public double kickerCurrentAmps = 0.0;
    }

    
}