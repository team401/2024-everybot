package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface CameraIO {
    public default PhotonPipelineResult getResult () {return new PhotonPipelineResult(); }
}
