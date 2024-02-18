package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        public double timestampSeconds = 0.0;
        public Pose2d estimatedVisionPose = new Pose2d();
        public double rotationToClosestTarget = 0.0;
        public boolean newResult = false;
        public boolean poseAvailable = false;
        public double averageTagDistanceM = 0.0;
        public int nTags = 0;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void updatePose(Pose2d simRobotPoseMeters) {}

    public default void set3dFieldSim(boolean enabled) {}
}
