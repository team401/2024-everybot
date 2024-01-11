package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveSim;

public class Drive extends SubsystemBase {
    private DifferentialDriveKinematics kinematics;
    // TODO: Odometry -> add auto log with advantage kit and inputs class
    private DifferentialDriveOdometry odometry;

    public Drive (DriveIO io) {
        kinematics = new DifferentialDriveKinematics(DriveSim.DriveConstants.TRACK_WIDTH);
        odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
    }

    public Pose2d getPose () {
        return odometry.getPoseMeters();
    }
    
    public void setPose (Pose2d newPose) {
        odometry.resetPosition(new Rotation2d(), 0, 0, newPose);
    }

    public void arcadeDrive (double xSpeed, double rotation) {
        var speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rotation));
        // TODO: Add feedforward and PID Controller before settting voltages or add in IO class
        // io.setVoltage (speeds.right * 12.0, speeds.left * 12.0)
    }

    @Override
    public void periodic() {
        odometry.update(new Rotation2d(), 0, 0); // placeholders
    }
}
