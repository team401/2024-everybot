package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;

public class Navigation {
    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator poseEstimator;
    private Vision vision;

    public Navigation (Vision vision) { // TODO: maybe initialize vision in Navigation unless its needed elsewhere
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
        this.vision = vision;
    }

    public void updateNav (Rotation2d gyro, double leftDistanceMeters, double rightDistanceMeters) {
        poseEstimator.update(gyro, leftDistanceMeters, rightDistanceMeters);
    }

    public void updateNavVision () {
        var result = vision.getResult();
        if (result.hasTargets()) {
            var cameraTimestamp = result.getTimestampSeconds();
            Pose2d visionMeasurePose2d = new Pose2d(); // placeholder until add targets
            poseEstimator.addVisionMeasurement(visionMeasurePose2d, cameraTimestamp);
        }
    }
}
