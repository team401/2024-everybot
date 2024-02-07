package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSimulation;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class Navigation extends SubsystemBase {
    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator poseEstimator;
    @AutoLogOutput private EstimatedRobotPose estimatedVisionPosition;
    private VisionIO vision;
    private VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private DoubleSupplier leftDistance;
    private DoubleSupplier rightDistance;
    private Supplier<Rotation2d> gyro;
    private Supplier<Pose2d> simulatedPose;
    private AprilTagFieldLayout layout;
    private Pose2d desiredTargetPose;
    @AutoLogOutput private Pose2d currentPose;

    public Navigation(
            DoubleSupplier leftDistance,
            DoubleSupplier rightDistance,
            Supplier<Rotation2d> gyro,
            Supplier<Pose2d> simulatedPose) { // REAL ROBOT
        switch (Constants.BotConstants.botMode) {
            case REAL:
                vision = new VisionIOReal();
                this.simulatedPose = null;
                break;
            case SIM:
                vision = new VisionIOSimulation();
                vision.set3dFieldSim(Constants.VisionConstants.SIM_FIELD_ENABLED);
                this.simulatedPose = simulatedPose;
                break;
            case REPLAY:
                vision = new VisionIOSimulation(); // ?
                this.simulatedPose = simulatedPose;
                break;
        }
        kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH);
        poseEstimator =
                new DifferentialDrivePoseEstimator(
                        kinematics,
                        new Rotation2d(),
                        0,
                        0,
                        new Pose2d(),
                        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5)));

        // set suppliers
        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;
        this.gyro = gyro;

        // set layout
        layout = Constants.FieldConstants.FIELD_LAYOUT;

        // set pose
        currentPose = new Pose2d();
    }

    private boolean robotInMidField() {
        return currentPose.getX() > FieldConstants.midfieldLowThresholdM
                && currentPose.getX() < FieldConstants.midfieldHighThresholdM;
    }
    private Matrix<N3, N1> cameraUncertainty(double averageTagDistanceM, int nTags) {
        /*
         * On this year's field, AprilTags are arranged into rough 'corridors' between the stage and
         * speaker, and a central 'desert,' where few tags can be found. It follows that we should
         * determine the variance of our camera measurements based on that.
         */
        if (nTags < 2) {
            return VisionConstants.singleTagUncertainty;
        } else if (averageTagDistanceM < 2.0 && this.robotInMidField()) {
            return VisionConstants.lowCameraUncertainty;
        } else {
            return VisionConstants.highCameraUncertainty;
        }
    }

    public void updateOdometry(
            Rotation2d gyro, double leftDistanceMeters, double rightDistanceMeters) {
        poseEstimator.update(gyro, leftDistanceMeters, rightDistanceMeters);
    }

    public void updateNavVision() {
        if (inputs.poseAvailable && inputs.newResult) {
            poseEstimator.addVisionMeasurement(inputs.estimatedVisionPose, inputs.timestampSeconds, cameraUncertainty(inputs.averageTagDistanceM, inputs.nTags));
        }
    }

    // returns heading error in radians
    public double getTargetHeading() {
        double currentHeading = getCurrentHeading();
        double robotVectorX = Math.cos(currentHeading);
        double robotVectorY = Math.sin(currentHeading);

        double targetVectorX =
                desiredTargetPose.getX() - poseEstimator.getEstimatedPosition().getX();
        double targetVectorY =
                desiredTargetPose.getY() - poseEstimator.getEstimatedPosition().getY();

        double dotProduct = (robotVectorX * targetVectorX) + (robotVectorY * targetVectorY);

        double targetVecMagnitude =
                Math.sqrt(Math.pow(targetVectorX, 2) + Math.pow(targetVectorY, 2));

        return Math.acos(
                dotProduct / targetVecMagnitude); // robot vector magnitude is one (unit vector)
    }

    public void setDesiredTarget(int targetId) {
        layout.getTagPose(targetId)
                .ifPresentOrElse(
                        (pose) -> { // should always be present tag id will be hardcoded in
                            // constants
                            desiredTargetPose = pose.toPose2d();
                        },
                        () -> {
                            desiredTargetPose = new Pose2d();
                        });
    }

    public double getCurrentHeading() {
        return currentPose.getRotation().getRadians();
    }

    public void updateCurrentPose() {
        currentPose = poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        updateOdometry(gyro.get(), leftDistance.getAsDouble(), rightDistance.getAsDouble());
        updateCurrentPose();
        vision.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
        if (Constants.BotConstants.botMode == Constants.Mode.REAL) {
            vision.updatePose(poseEstimator.getEstimatedPosition());
        } else {
            vision.updatePose(simulatedPose.get());
        }
        updateNavVision();
    }
}
