package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ScoringIOReal implements ScoringIO{
    
    private final TalonFX kicker = new TalonFX(0); //placeholders

    private final TalonFX leftShooter = new TalonFX(0); //placeholders
    private final TalonFX rightShooter = new TalonFX(0); //placeholders

    private double goalLeftVelocityRPM = 0.0;
    private double goalRightVelocityRPM = 0.0;

    public ScoringIOReal(){
        leftShooter.setNeutralMode(NeutralModeValue.Coast); //placeholders
        rightShooter.setNeutralMode(NeutralModeValue.Coast); //placeholders
    }

    public void setShooterVolts(double volts){
        leftShooter.setVoltage(volts); //placeholders
        rightShooter.setVoltage(volts); //placeholders
    }

    @Override
    public void updateInputs(ScoringIOInputs inputs){

    }


}
