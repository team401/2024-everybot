package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants;

public class CameraIOReal implements CameraIO {
    private PhotonCamera cam;

    public CameraIOReal() {
        cam = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
    }

    public PhotonPipelineResult getResult() {
        return cam.getLatestResult();
    }
}
