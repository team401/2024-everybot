package frc.robot.subsystems.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ScoringConstants;

public class ShooterIOSim implements ShooterIO {
    private FlywheelSim leftSim = new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.00001);
    private FlywheelSim rightSim = new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.00001);

    private double goalLeftVelocityRPM = 0.0;
    private double goalRightVelocityRPM = 0.0;
    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double kickerVolts = 0.0;

    private PIDController leftController =
            new PIDController(ScoringConstants.kP, ScoringConstants.kI, ScoringConstants.kD);
    private PIDController rightController =
            new PIDController(ScoringConstants.kP, ScoringConstants.kI, ScoringConstants.kD);

    private SimpleMotorFeedforward shooterFF =
            new SimpleMotorFeedforward(
                    ScoringConstants.kS, ScoringConstants.kV, ScoringConstants.kA);

    public void setShooterVolts(double volts) {
        leftSim.setInputVoltage(volts);
        rightSim.setInputVoltage(volts);
    }

    public void setKickerVolts(double volts) {
        kickerVolts = volts;
    }

    public void setShooterVelocity(double velocity) {
        goalLeftVelocityRPM = velocity;
        goalRightVelocityRPM = velocity;

        leftAppliedVolts =
                leftController.calculate(leftSim.getAngularVelocityRPM(), goalLeftVelocityRPM)
                        + shooterFF.calculate(leftSim.getAngularVelocityRPM());
        rightAppliedVolts =
                rightController.calculate(rightSim.getAngularVelocityRPM(), goalRightVelocityRPM)
                        + shooterFF.calculate(rightSim.getAngularVelocityRPM());
        leftSim.setInputVoltage(leftAppliedVolts);
        rightSim.setInputVoltage(rightAppliedVolts);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftSim.update(0.02);
        rightSim.update(0.02);

        inputs.leftShooterAppliedVolts = leftAppliedVolts;
        inputs.leftShooterCurrentAmps = leftSim.getCurrentDrawAmps();
        inputs.leftShooterVelocityRPM = leftSim.getAngularVelocityRPM();

        inputs.rightShooterAppliedVolts = rightAppliedVolts;
        inputs.rightShooterCurrentAmps = rightSim.getCurrentDrawAmps();
        inputs.rightShooterVelocityRPM = rightSim.getAngularVelocityRPM();

        inputs.kickerAppliedVolts = kickerVolts;
    }
}
