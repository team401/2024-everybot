package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.SwerveConstants;
import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveHardwareIO implements SwerveIO {

    private SwerveDrive swerveDrive;

    public SwerveHardwareIO() {
        try {
            swerveDrive =
                    new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                            .createSwerveDrive(Units.feetToMeters(SwerveConstants.MAX_SPEED));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
    }

    @Override
    public void updateInputs(SwerveInputs inputs) {}

    @Override
    public void updateOutputs() {}

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(
                translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public double getDriveBaseRadiusMeters() {
        return swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters();
    }
}
