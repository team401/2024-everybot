package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.driveTrainState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

    private double vx, vy, omega = 0.0;
    private final DriveIO io;
    private driveTrainState mode;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final SimpleMotorFeedforward driveff =
            new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV);

    private double forward;
    private double rotation;

    public Drive(DriveIO io) {
        this.io = io;
        configurePathPlanner();
        // Configure the AutoBuilder last
        AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            this // Reference to this subsystem to set requirements
        );
    }
    

    public void configurePathPlanner(){
        double driveBaseRadius = 0;
        }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
        controlDriveTrain();
    }

    public void setArcadeDrive(double forward, double rotation, driveTrainState mode) {
        this.forward = forward;
        this.rotation = rotation;
        this.mode = mode;
    }

    /** Run open loop based on stick positions. */
    public void driveArcade(double xSpeed, double zRotation) {
        var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
        io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
    }

    /** Stops the drive. */
    public void stop() {
        io.setVoltage(0.0, 0.0);
    }

    public Rotation2d getGyroRotation2d() {
        return inputs.gyroYaw;
    }

    /** Returns the position of the left wheels in meters. */
    @AutoLogOutput
    public double getLeftPositionMeters() {
        return inputs.leftPositionRad * Constants.DriveConstants.WHEEL_RADIUS;
    }

    /** Returns the position of the right wheels in meters. */
    @AutoLogOutput
    public double getRightPositionMeters() {
        return inputs.rightPositionRad * Constants.DriveConstants.WHEEL_RADIUS;
    }

    /** Returns the velocity of the left wheels in meters/second. */
    @AutoLogOutput
    public double getLeftVelocityMetersPerSec() {
        return inputs.leftVelocityRadPerSec * Constants.DriveConstants.WHEEL_RADIUS;
    }

    /** Returns the velocity of the right wheels in meters/second. */
    @AutoLogOutput
    public double getRightVelocityMetersPerSec() {
        return inputs.rightVelocityRadPerSec * Constants.DriveConstants.WHEEL_RADIUS;
    }

    public Pose2d getSimulatedPose() {
        return inputs.simulatedPose;
    }

    public void updateMode(driveTrainState mode) {
        this.mode = mode;
    }

    public void controlDriveTrain() {
        switch (mode) {
            case MANUAL:
                this.driveArcade(forward, rotation);
                break;
            case AIM:
                this.stop();
                break;
            case AUTO:
                this.getAutoPath("Center 3pc");
            default:
                this.stop();
                break;
        }
    }
}
