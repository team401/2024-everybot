package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class VisionIOReal implements VisionIO {
    public PhotonCamera camera;
    private PhotonPoseEstimator cameraPoseEstimator;
    private AprilTagFieldLayout layout;
    private double lastEstTimestamp = 0;

    public VisionIOReal () {
        camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println(e);
        }
        Transform3d robotToCamera = new Transform3d(Constants.VisionConstants.BOT_TO_CAM_TRL, Constants.VisionConstants.BOT_TO_CAMERA_ROT);
        cameraPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamera);
    }

    public void updateInputs (VisionIOInputs inputs) {
        PhotonPipelineResult result = camera.getLatestResult();
        inputs.timestampSeconds = result.getTimestampSeconds();

        cameraPoseEstimator.update().ifPresentOrElse(
            est -> {
                inputs.estimatedVisionPose = est.estimatedPose.toPose2d();
            }, () -> {
                inputs.estimatedVisionPose = null;
            });
    }

    public void updatePose (Pose2d drivetrainPoseMeters) {
        cameraPoseEstimator.setReferencePose(drivetrainPoseMeters);
    }
}
