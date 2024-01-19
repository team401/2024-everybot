package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveIO {

    @AutoLog
    public static class DriveIOInputs{
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};

        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};

        public Rotation2d gyroYaw = new Rotation2d();

        public Pose2d simulatedPose = new Pose2d(); // only update in sim
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(DriveIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double leftVolts, double rightVolts) {}

    /** Run closed loop at the specified velocity. */
    public default void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {}
    
    
}
