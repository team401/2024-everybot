package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        public double timestampSeconds = 0.0;
        public double[] cornerXArr = new double[] {};
        public double[] cornerYArr = new double[] {};
        public Pose2d estimatedVisionPose = new Pose2d();
    }

    public default void updateInputs (VisionIOInputs inputs) {}

    public default void updatePose (Pose2d simRobotPoseMeters) {}
}