package frc.robot.subsystems.vision;

import java.io.IOException;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;

public class VisionSimulation {
    private VisionSystemSim sim;
    private TargetModel targetModel;
    private AprilTagFieldLayout tagLayout;
    private SimCameraProperties properties;

    public VisionSimulation (PhotonCameraSim simulatedCamera) {
        // create simulated field
        sim = new VisionSystemSim(Constants.VisionConstants.CAMERA_NAME);
        targetModel = TargetModel.kAprilTag36h11;
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println(e); // quit program ?
        }
        sim.addAprilTags(tagLayout);
        
        // set up camera
        Transform3d robotToCamera = new Transform3d(Constants.VisionConstants.BOT_TO_CAM_TRL, Constants.VisionConstants.BOT_TO_CAMERA_ROT);
        sim.addCamera(simulatedCamera, robotToCamera);
    }

    public void updateFromDriveSim (Pose2d simRobotPoseMeters) {
        sim.update(simRobotPoseMeters);
    }

    public Field2d getField() {
        return sim.getDebugField();
    }
}
