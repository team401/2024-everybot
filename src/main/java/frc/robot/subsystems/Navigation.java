package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSimulation;

public class Navigation extends SubsystemBase {
    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator poseEstimator;
    private VisionIO vision;
    private DoubleSupplier leftDistance;
    private DoubleSupplier rightDistance;
    private Supplier<Rotation2d> gyro;

    public Navigation (DoubleSupplier leftDistance, DoubleSupplier rightDistance, Supplier<Rotation2d> gyro) {  // REAL ROBOT
        switch(Constants.BotConstants.botMode) {
            case REAL:
                vision = new VisionIOReal();
                break;
            case SIM:
                vision = new VisionIOSimulation();
                break;
            case REPLAY:
                vision = new VisionIOSimulation(); // ?
                break;
        }
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

       // set suppliers
       this.leftDistance = leftDistance;
       this.rightDistance = rightDistance;
       this.gyro = gyro;
    }

    public void updateOdometry (Rotation2d gyro, double leftDistanceMeters, double rightDistanceMeters) {
        poseEstimator.update(gyro, leftDistanceMeters, rightDistanceMeters);
    }

    public void updateNavVision () {
       vision.getEstimatedPosition().ifPresentOrElse(
            est -> {
                poseEstimator.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
            }, () -> {
                // do nothing ( no measurement )
            });
    }

    @Override
    public void periodic () {
        updateOdometry(
            gyro.get(), 
            leftDistance.getAsDouble(), 
            rightDistance.getAsDouble()
        );
        vision.update(poseEstimator.getEstimatedPosition());
        if(Constants.BotConstants.botMode == Constants.Mode.REAL) {
            updateNavVision(); // photon lib says sim camera cannot use photon pose estimator
        }
    }
}
