package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

    // TODO: Odometry -> add auto log with advantage kit and inputs class



    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
    private DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV);

    public Drive(DriveIO io) {
        this.io = io;
        //(DriveConstants.frontLeftID, DriveConstants.frontRightID, DriveConstants.backLeftID, DriveConstants.backRightID);

        kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH);
        odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        // Update odometry  
        odometry.update(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters());
    }

    public void arcadeDrive (double xSpeed, double rotation) {
        var speeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rotation));
        // TODO: Add feedforward and PID Controller before settting voltages or add in IO class
        // io.setVoltage (speeds.right * 12.0, speeds.left * 12.0)
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
        feedforward.calculate(leftRadPerSec),
        feedforward.calculate(rightRadPerSec));
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

  /** Returns the current odometry pose in meters. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters(), pose);
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
}
