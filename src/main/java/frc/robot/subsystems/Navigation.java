package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;

public class Navigation {
    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator poseEstimator;
    private PhotonPoseEstimator cameraPoseEstimator;
    private Vision vision;
    private AprilTagFieldLayout layout;

    public Navigation () {  // REAL ROBOT
        vision = new Vision();
        kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH);
        poseEstimator = new DifferentialDrivePoseEstimator(
            kinematics,
            new Rotation2d(), 
            0, 
            0, 
            new Pose2d(), 
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), 
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5))
        );
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println(e);
        }
        Transform3d robotToCamera = new Transform3d(Constants.VisionConstants.BOT_TO_CAM_TRL, Constants.VisionConstants.BOT_TO_CAMERA_ROT);
        cameraPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, vision.getCamera(), robotToCamera);

        // TODO: Add simulated navigation (already built vision simulation)
    }

    public void updateNav (Rotation2d gyro, double leftDistanceMeters, double rightDistanceMeters) {
        poseEstimator.update(gyro, leftDistanceMeters, rightDistanceMeters);
        cameraPoseEstimator.setReferencePose(poseEstimator.getEstimatedPosition());
        updateNavVision();
    }

    public void updateNavVision () {
        EstimatedRobotPose estimatedVisionPose = cameraPoseEstimator.update().orElseThrow();
        poseEstimator.addVisionMeasurement(estimatedVisionPose.estimatedPose.toPose2d(), estimatedVisionPose.timestampSeconds);
    }
}
