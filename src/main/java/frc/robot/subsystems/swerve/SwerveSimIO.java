package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSimIO implements SwerveIO {

    private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();
    private Pose2d pose;
    private double gyroAngle = 0.0;
    private boolean brakeMode = false;

    // Sim constants.
    private final double SIM_UPDATE_DT = 0.05;

    // Simulated hardware.
    private final double SIM_MAX_SPEED = 4.5;
    private final int NUM_MODULES = 4;
    Translation2d[] moduleOffsets = new Translation2d[NUM_MODULES];
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[NUM_MODULES];

    // Physical dimensions (hardcoded for now, need to fork/update YAGSL to make
    // this better).
    private final double X_METERS = 0.32;
    private final double Y_METERS = 0.218;
    private final double DRIVE_RADIUS_METERS = 0.387;

    // Pose estimation.
    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[NUM_MODULES];
    SwerveDriveKinematics swerveKinematics;
    SwerveDrivePoseEstimator poseEstimator;

    public SwerveSimIO() {
        for (int i = 0; i < NUM_MODULES; ++i) {
            double x_sign = (i > 1) ? 1 : -1;
            double y_sign = (i % 2 == 0) ? 1 : -1;
            moduleOffsets[i] = new Translation2d(x_sign * X_METERS, y_sign * Y_METERS);

            swerveModulePositions[i] = new SwerveModulePosition();
        }
        swerveKinematics = new SwerveDriveKinematics(moduleOffsets);
        pose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        swerveKinematics, new Rotation2d(0.0), swerveModulePositions, pose);
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {}

    @Override
    public void updateOutputs() {
        // Update the swerve module states immediately to the goal speeds. Ensure the
        // maximum speed is not exceeded.
        swerveModuleStates = swerveKinematics.toSwerveModuleStates(robotChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SIM_MAX_SPEED);

        // Integrate the wheel positions and update the angles.
        for (int i = 0; i < NUM_MODULES; ++i) {
            swerveModulePositions[i].angle = swerveModuleStates[i].angle;
            swerveModulePositions[i].distanceMeters +=
                    swerveModuleStates[i].speedMetersPerSecond * SIM_UPDATE_DT;
        }

        // Update gyro angle.
        double omega = getFieldVelocity().omegaRadiansPerSecond;
        gyroAngle += omega * SIM_UPDATE_DT;

        // Update pose estimate.
        poseEstimator.update(Rotation2d.fromRadians(gyroAngle), swerveModulePositions);

        // Estimate the position.
        pose = poseEstimator.getEstimatedPosition();
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds speeds =
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                pose.getRotation())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        setChassisSpeeds(speeds);
    }

    @Override
    public void driveFieldOriented(ChassisSpeeds velocity) {
        Translation2d translation2d =
                new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
        drive(translation2d, velocity.omegaRadiansPerSecond, true);
    }

    @Override
    public void resetOdometry(Pose2d initialHolonomicPose) {
        poseEstimator.resetPosition(
                Rotation2d.fromRadians(gyroAngle), swerveModulePositions, initialHolonomicPose);
        pose = poseEstimator.getEstimatedPosition();
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        robotChassisSpeeds = chassisSpeeds;
    }

    @Override
    public void zeroGyro() {
        gyroAngle = 0.0;
    }

    @Override
    public void setMotorBrake(boolean brake) {
        brakeMode = brake;
    }

    @Override
    public ChassisSpeeds getFieldVelocity() {
        ChassisSpeeds fieldChassisSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(robotChassisSpeeds, pose.getRotation());
        return fieldChassisSpeeds;
    }

    @Override
    public ChassisSpeeds getRobotVelocity() {
        return robotChassisSpeeds;
    }

    @Override
    public double getDriveBaseRadiusMeters() {
        return DRIVE_RADIUS_METERS;
    }
}
