package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOReal implements VisionIO {
    public PhotonCamera camera;
    private PhotonPoseEstimator cameraPoseEstimator;
    private AprilTagFieldLayout layout;
    private double lastEstTimestamp = 0.0;

    public VisionIOReal() {
        camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
        layout = Constants.FieldConstants.FIELD_LAYOUT;
        Transform3d robotToCamera =
                new Transform3d(
                        Constants.VisionConstants.BOT_TO_CAM_TRL,
                        Constants.VisionConstants.BOT_TO_CAMERA_ROT);
        cameraPoseEstimator =
                new PhotonPoseEstimator(
                        layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamera);
    }

    public void updateInputs(VisionIOInputs inputs) {
        PhotonPipelineResult result = camera.getLatestResult();
        inputs.timestampSeconds = result.getTimestampSeconds();

        boolean newResult = Math.abs(result.getTimestampSeconds() - lastEstTimestamp) > 1e-5;

        cameraPoseEstimator
                .update()
                .ifPresentOrElse(
                        est -> {
                            if (newResult) {
                                lastEstTimestamp =
                                        result.getTimestampSeconds(); // update timestamp since
                                // frame has been added
                                inputs.estimatedVisionPose = est.estimatedPose.toPose2d();
                            } else {
                                inputs.estimatedVisionPose =
                                        null; // out of date (already added to pose estimator)
                            }
                        },
                        () -> {
                            inputs.estimatedVisionPose = null;
                        });
        cameraPoseEstimator.update().ifPresentOrElse(
            est -> {
                if(newResult) {
                    lastEstTimestamp = result.getTimestampSeconds(); // update timestamp since frame has been added
                    inputs.estimatedVisionPose = est.estimatedPose.toPose2d();
                } else {
                    inputs.estimatedVisionPose = null; // out of date (already added to pose estimator)
                }
            }, () -> {
                inputs.estimatedVisionPose = null;
            });
        
        if(result.hasTargets()) {
            inputs.rotationToClosestTarget = result.getBestTarget().getYaw();
        } else {
            inputs.rotationToClosestTarget = 0.0; // dont move
        }
    }

    public void updatePose(Pose2d drivetrainPoseMeters) {
        cameraPoseEstimator.setReferencePose(drivetrainPoseMeters);
    }
}
