package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

    private final DriveIO io;
    private DriveTrainState mode;
    private DoubleSupplier targetHeading;
    private DoubleSupplier currentHeading;
    @AutoLogOutput private boolean aligned = false;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private double forward;
    @AutoLogOutput private double rotation;

    public Drive(DriveIO io) {
        this.io = io;
        this.targetHeading =
                () -> {
                    return 0.0;
                };
        this.currentHeading =
                () -> {
                    return 0.0;
                };
        this.mode = DriveTrainState.MANUAL;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
        controlDriveTrain();
    }

    public void setDriveState(DriveTrainState state) {
        mode = state;
        if (state == DriveTrainState.AIM) {
            aligned = false; // reset aim
        }
    }

    public void setCurrentHeadingSupplier(DoubleSupplier currentHeading) {
        this.currentHeading = currentHeading;
    }

    public void setTargetHeadingSupplier(DoubleSupplier targetHeading) {
        this.targetHeading = targetHeading;
    }

    public void setArcadeDrive(double forward, double rotation) {
        this.forward = forward;
        this.rotation = rotation;
        mode = DriveTrainState.MANUAL;
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

    public void aim() {
        this.forward = 0;
        this.rotation = targetHeading.getAsDouble();
        this.driveArcade(forward, rotation);
        if (Math.abs(currentHeading.getAsDouble() - targetHeading.getAsDouble()) < 1e-5) {
            aligned = true;
        }
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

    public boolean isAligned() {
        return aligned;
    }

    public void controlDriveTrain() {
        switch (mode) {
            case MANUAL:
                this.driveArcade(forward, rotation);
                break;
            case AIM:
                this.aim();
                break;
            default:
                this.stop();
                break;
        }
    }
}
