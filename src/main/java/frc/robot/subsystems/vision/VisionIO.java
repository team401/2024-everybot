package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    public default PhotonCamera getCamera () { return new PhotonCamera(""); }

    public default void update (Pose2d simRobotPoseMeters) {}

    public default PhotonPipelineResult getResult () { return new PhotonPipelineResult(); };

    public default EstimatedRobotPose getEstimatedPosition () { return new EstimatedRobotPose(null, 0, null, null); }
}