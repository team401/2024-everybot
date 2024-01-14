package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;

public class Vision {
    public PhotonCamera camera;
    private PhotonCameraSim simulatedCamera;
    private VisionSimulation simulatedVision;

    public Vision () {
        if(RobotBase.isReal()) {
            camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
        } else {
            camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
            SimCameraProperties properties =  new SimCameraProperties();
            properties.setCalibration(
                Constants.VisionConstants.RESOLUTION_WIDTH, 
                Constants.VisionConstants.RESOLUTION_HEIGHT, 
                Rotation2d.fromDegrees(Constants.VisionConstants.CAM_DIAG_FOV)
            );
            properties.setFPS(Constants.VisionConstants.CAMERA_FPS);
            simulatedCamera = new PhotonCameraSim(camera, properties);
            simulatedVision = new VisionSimulation(simulatedCamera);
        }
    }

    public PhotonPipelineResult getResult () {
        return camera.getLatestResult();
    }

    public void updateSim (Pose2d drivetrainPoseMeters) {
        simulatedVision.updateFromDriveSim(drivetrainPoseMeters);
    }
    public Field2d getSimulatedField () {
        return simulatedVision.getField();
    }
    public PhotonCamera getCamera() {
        return camera;
    }
}
