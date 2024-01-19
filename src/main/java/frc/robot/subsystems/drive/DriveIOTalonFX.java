package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class DriveIOTalonFX implements DriveIO{

    private final TalonFX driveMotorFrontLeft = new TalonFX(0);
    private final TalonFX driveMotorFrontRight  = new TalonFX(1);
    private final TalonFX driveMotorBackLeft = new TalonFX(2);
    private final TalonFX driveMotorBackRight = new TalonFX(3);

    private final StatusSignal<Double> leftPosition = driveMotorFrontLeft.getPosition();
    private final StatusSignal<Double> leftVelocity = driveMotorFrontLeft.getVelocity();
    private final StatusSignal<Double> leftAppliedVolts = driveMotorFrontLeft.getMotorVoltage();
    private final StatusSignal<Double> leftLeaderCurrent = driveMotorFrontLeft.getStatorCurrent();
    private final StatusSignal<Double> leftFollowerCurrent = driveMotorBackLeft.getStatorCurrent();

    private final StatusSignal<Double> rightPosition = driveMotorFrontRight.getPosition();
    private final StatusSignal<Double> rightVelocity = driveMotorFrontRight.getVelocity();
    private final StatusSignal<Double> rightAppliedVolts = driveMotorFrontRight.getMotorVoltage();
    private final StatusSignal<Double> rightLeaderCurrent = driveMotorFrontRight.getStatorCurrent();
    private final StatusSignal<Double> rightFollowerCurrent = driveMotorBackRight.getStatorCurrent();

    private final Pigeon2 pigeon = new Pigeon2(Constants.DriveConstants.pigeonID);
    private final StatusSignal<Double> yaw = pigeon.getYaw();


    public DriveIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 60.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = Constants.DriveConstants.kP;
    config.Slot0.kD = Constants.DriveConstants.kD;
    driveMotorFrontLeft.getConfigurator().apply(config);
    driveMotorBackLeft.getConfigurator().apply(config);
    driveMotorFrontRight.getConfigurator().apply(config);
    driveMotorBackRight.getConfigurator().apply(config);
    driveMotorBackLeft.setControl(new Follower(driveMotorFrontLeft.getDeviceID(), false));
    driveMotorBackRight.setControl(new Follower(driveMotorFrontRight.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, leftPosition, rightPosition, yaw); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent);
    driveMotorFrontLeft.optimizeBusUtilization();
    driveMotorBackLeft.optimizeBusUtilization();
    driveMotorFrontRight.optimizeBusUtilization();
    driveMotorBackRight.optimizeBusUtilization();
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftLeaderCurrent,
        leftFollowerCurrent,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightLeaderCurrent,
        rightFollowerCurrent,
        yaw);

    inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble()) / Constants.DriveConstants.GEAR_RATIO;
    inputs.leftVelocityRadPerSec =
        Units.rotationsToRadians(leftVelocity.getValueAsDouble()) / Constants.DriveConstants.GEAR_RATIO;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftCurrentAmps =
        new double[] {leftLeaderCurrent.getValueAsDouble(), leftFollowerCurrent.getValueAsDouble()};

    inputs.rightPositionRad =
        Units.rotationsToRadians(rightPosition.getValueAsDouble()) / Constants.DriveConstants.GEAR_RATIO;
    inputs.rightVelocityRadPerSec =
        Units.rotationsToRadians(rightVelocity.getValueAsDouble()) / Constants.DriveConstants.GEAR_RATIO;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps =
        new double[] {
          rightLeaderCurrent.getValueAsDouble(), rightFollowerCurrent.getValueAsDouble()
        };

    inputs.gyroYaw = Rotation2d.fromDegrees(yaw.getValueAsDouble());

    inputs.simulatedPose = null;
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    driveMotorFrontLeft.setControl(new VoltageOut(leftVolts));
    driveMotorFrontRight.setControl(new VoltageOut(rightVolts));
  }

  @Override
  public void setVelocity(
    double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    driveMotorFrontLeft.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(leftRadPerSec * Constants.DriveConstants.GEAR_RATIO),
            0.0,
            true,
            leftFFVolts,
            0,
            false,
            false,
            false));
    driveMotorFrontRight.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(rightRadPerSec * Constants.DriveConstants.GEAR_RATIO),
            0.0,
            true,
            rightFFVolts,
            0,
            false,
            false,
            false));
  }
    
}
