package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;

public class Vision {
    public PhotonCamera camera;

    public Vision () {
        if(RobotBase.isReal()) {
            camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
        } // else in sim
    }

    public PhotonPipelineResult getResult () {
        return camera.getLatestResult();
    }
}
