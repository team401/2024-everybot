package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final SimpleMotorFeedforward driveff =
            new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV);

    public Drive(DriveIO io) {
        this.io = io;
        // (DriveConstants.frontLeftID, DriveConstants.frontRightID, DriveConstants.backLeftID,
        // DriveConstants.backRightID);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
    }

    /** Run open loop at the specified voltage. */
    public void driveVolts(double leftVolts, double rightVolts) {
        io.setVoltage(leftVolts, rightVolts);
    }

    /** Run closed loop at the specified voltage. */
    public void driveVelocity(double leftMetersPerSec, double rightMetersPerSec) {
        Logger.recordOutput("Drive/LeftVelocitySetpointMetersPerSec", leftMetersPerSec);
        Logger.recordOutput("Drive/RightVelocitySetpointMetersPerSec", rightMetersPerSec);
        double leftRadPerSec = leftMetersPerSec / Constants.DriveConstants.WHEEL_RADIUS;
        double rightRadPerSec = rightMetersPerSec / Constants.DriveConstants.WHEEL_RADIUS;
        io.setVelocity(
                leftRadPerSec,
                rightRadPerSec,
                driveff.calculate(leftRadPerSec),
                driveff.calculate(rightRadPerSec));
    }

    /** Run open loop based on stick positions. */
    // public void driveArcade(double xSpeed, double zRotation) {
    //   var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
    //   io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
    // }

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

    /** Returns the average velocity in radians/second. */
    public double getCharacterizationVelocity() {
        return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
    }

    public Pose2d getSimulatedPose() {
        return inputs.simulatedPose;
    }

    public void setDiffDriveControls(double forward, double rotation) {
        io.setVoltage(forward, rotation); // idk if this is right
    }
}
