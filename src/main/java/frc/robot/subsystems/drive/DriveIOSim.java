package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimConstants;

public class DriveIOSim implements DriveIO{

    private DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean closedLoop = false;
  private PIDController leftPID = new PIDController(SimConstants.KP, 0.0, SimConstants.KD);
  private PIDController rightPID = new PIDController(SimConstants.KP, 0.0, SimConstants.KD);
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;


  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts =
          MathUtil.clamp(
              leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / DriveConstants.WHEEL_RADIUS)
                  + leftFFVolts,
              -12.0,
              12.0);
      rightAppliedVolts =
          MathUtil.clamp(
              leftPID.calculate(sim.getRightVelocityMetersPerSecond() / DriveConstants.WHEEL_RADIUS)
                  + rightFFVolts,
              -12.0,
              12.0);
      sim.setInputs(leftAppliedVolts, rightAppliedVolts);
    }

    sim.update(0.02);
    inputs.leftPositionRad = sim.getLeftPositionMeters() / DriveConstants.WHEEL_RADIUS;
    inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / DriveConstants.WHEEL_RADIUS;
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {sim.getLeftCurrentDrawAmps()};

    inputs.rightPositionRad = sim.getRightPositionMeters() / DriveConstants.WHEEL_RADIUS;
    inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / DriveConstants.WHEEL_RADIUS;
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {sim.getRightCurrentDrawAmps()};

    inputs.gyroYaw = sim.getHeading();

    inputs.simulatedPose = sim.getPose();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
    sim.setInputs(leftAppliedVolts, rightAppliedVolts);
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    leftPID.setSetpoint(leftRadPerSec);
    rightPID.setSetpoint(rightRadPerSec);
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
  }
    
}
