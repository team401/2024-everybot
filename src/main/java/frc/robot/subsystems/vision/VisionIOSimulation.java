package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSimulation implements VisionIO {
    public PhotonCamera camera;
    private PhotonCameraSim simulatedCamera;
    private VisionSystemSim sim;
    private PhotonPoseEstimator cameraPoseEstimator;
    private AprilTagFieldLayout layout;
    private double lastEstTimestamp = 0;

    public VisionIOSimulation() {
        // create simulated field
        sim = new VisionSystemSim(Constants.VisionConstants.CAMERA_NAME);
        layout = Constants.FieldConstants.FIELD_LAYOUT;
        sim.addAprilTags(layout);

        // set up camera
        camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);
        SimCameraProperties properties = new SimCameraProperties();
        properties.setCalibration(
                Constants.VisionConstants.RESOLUTION_WIDTH,
                Constants.VisionConstants.RESOLUTION_HEIGHT,
                Rotation2d.fromDegrees(Constants.VisionConstants.CAM_DIAG_FOV));
        properties.setFPS(Constants.VisionConstants.CAMERA_FPS);
        simulatedCamera = new PhotonCameraSim(camera, properties);
        Transform3d robotToCamera =
                new Transform3d(
                        Constants.VisionConstants.BOT_TO_CAM_TRL,
                        Constants.VisionConstants.BOT_TO_CAMERA_ROT);
        sim.addCamera(simulatedCamera, robotToCamera);

        // set up pose estimator
        cameraPoseEstimator =
                new PhotonPoseEstimator(
                        layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamera);
        cameraPoseEstimator.setReferencePose(new Pose2d());
    }

    public void updateInputs(VisionIOInputs inputs) {
        PhotonPipelineResult result = camera.getLatestResult();
        inputs.timestampSeconds = result.getTimestampSeconds();

        var estimate = cameraPoseEstimator.update();
        double latestTimestamp = result.getTimestampSeconds();
        inputs.newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        estimate.ifPresentOrElse(
                est -> {
                    inputs.poseAvailable = true;
                    if (inputs.newResult) {
                        getField()
                                .getObject("VisionEstimation")
                                .setPose(est.estimatedPose.toPose2d());
                        inputs.estimatedVisionPose = est.estimatedPose.toPose2d();
                        inputs.averageTagDistanceM = calculateAverageTagDistance(est);
                        inputs.nTags = est.targetsUsed.size();
                    }
                },
                () -> {
                    if (inputs.newResult) {
                        getField().getObject("VisionEstimation").setPoses();
                    }
                    inputs.poseAvailable = false;
                });
        if (inputs.newResult) {
            lastEstTimestamp = latestTimestamp;
        }
    }

    public Field2d getField() {
        return sim.getDebugField();
    }

    public void updatePose(Pose2d simRobotPoseMeters) {
        sim.update(simRobotPoseMeters);
    }

    public void set3dFieldSimActive(boolean enabled) {
        simulatedCamera.enableDrawWireframe(enabled);
    }

    private static double calculateAverageTagDistance(EstimatedRobotPose pose) {
        double distance = 0.0;
        for (PhotonTrackedTarget target : pose.targetsUsed) {
            distance +=
                    target.getBestCameraToTarget()
                            .getTranslation()
                            .getDistance(new Translation3d());
        }
        distance /= pose.targetsUsed.size();

        return distance;
    }
}
