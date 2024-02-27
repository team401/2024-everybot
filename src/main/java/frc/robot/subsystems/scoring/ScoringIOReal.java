package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ScoringConstants;

public class ScoringIOReal implements ScoringIO {

    private final TalonFX kicker = new TalonFX(ScoringConstants.kickerId);

    private final TalonFX leftShooter = new TalonFX(ScoringConstants.leftShooterId);
    private final TalonFX rightShooter = new TalonFX(ScoringConstants.rightShooterId);

    private double goalLeftVelocityRPM = 0.0;
    private double goalRightVelocityRPM = 0.0;

    public ScoringIOReal() {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.withKP(ScoringConstants.kP);
        slot0.withKI(ScoringConstants.kI);
        slot0.withKD(ScoringConstants.kD);
        slot0.withKS(ScoringConstants.kS);
        slot0.withKV(ScoringConstants.kV);
        slot0.withKA(ScoringConstants.kA);

        leftShooter.setNeutralMode(NeutralModeValue.Brake);
        rightShooter.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfigurator leftConfig = leftShooter.getConfigurator();
        TalonFXConfigurator rightConfig = rightShooter.getConfigurator();
        TalonFXConfigurator kickerConfig = kicker.getConfigurator();
        leftConfig.apply(
                new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(ScoringConstants.leftCurrentLimit)
                        .withSupplyCurrentLimitEnable(true));
        rightConfig.apply(
                new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(ScoringConstants.rightCurrentLimit)
                        .withSupplyCurrentLimitEnable(true));
        kickerConfig.apply(
                new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(ScoringConstants.kickerCurrentlimit)
                        .withSupplyCurrentLimitEnable(true));

        leftShooter.getConfigurator().apply(slot0);
        rightShooter.getConfigurator().apply(slot0);
    }

    public void setShooterVelocity (double velocity) {
        goalLeftVelocityRPM = velocity;
        goalRightVelocityRPM = velocity;

        if(velocity == 0.0) {
            setShooterVolts(0.0);
        } else {
            leftShooter.setControl(new VelocityDutyCycle(goalLeftVelocityRPM / 60.0));
            rightShooter.setControl(new VelocityDutyCycle(goalRightVelocityRPM / 60.0));
        }
    }

    public void setShooterVolts(double volts) {
        leftShooter.setControl(new VoltageOut(goalLeftVelocityRPM));
        rightShooter.setControl(new VoltageOut(goalRightVelocityRPM));
    }

    public void setKickerVolts(double volts) {
        kicker.setVoltage(volts);
    }

    @Override
    public void updateInputs(ScoringIOInputs inputs) {
        inputs.leftShooterVelocityRPM = leftShooter.getVelocity().getValueAsDouble() * 60.0;
        inputs.leftShooterAppliedVolts = leftShooter.getMotorVoltage().getValueAsDouble();
        inputs.leftShooterVolts = leftShooter.getSupplyVoltage().getValueAsDouble();
        inputs.leftShooterCurrentAmps = leftShooter.getStatorCurrent().getValueAsDouble();

        inputs.rightShooterVelocityRPM = rightShooter.getVelocity().getValueAsDouble() * 60.0;
        inputs.rightShooterAppliedVolts = rightShooter.getMotorVoltage().getValueAsDouble();
        inputs.rightShooterVolts = rightShooter.getSupplyVoltage().getValueAsDouble();
        inputs.rightShooterCurrentAmps = rightShooter.getStatorCurrent().getValueAsDouble();

        inputs.kickerAppliedVolts = kicker.getMotorVoltage().getValueAsDouble();
        inputs.kickerCurrentAmps = kicker.getStatorCurrent().getValueAsDouble();
    }
}
