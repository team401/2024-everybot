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

    private final DriveIO io;
    private driveTrainState mode;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final SimpleMotorFeedforward driveff =
            new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV);

    private double forward;
    private double rotation;

    Trajectory.State goal = trajectory.sample(3.4); // sample the trajectory at 3.4 seconds from the beginning
    ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);

    RamseteController controller1 = new RamseteController();
    ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
    double left = wheelSpeeds.leftMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond;

    public Drive(DriveIO io) {
        this.io = io;
        configurePathPlanner();
    }

    public void configurePathPlanner() {
        double driveBaseRadius = 0;

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) ->
                        this.setGoalChassisSpeeds(
                                speeds, true), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> false, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
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
