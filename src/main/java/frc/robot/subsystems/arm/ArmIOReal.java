package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ArmConstants;

public class ArmIOReal implements ArmIO {

    ArmIOInputs armIOInputs = new ArmIOInputs();
    CANSparkMax leftMotor = new CANSparkMax(ArmConstants.leftMotorID, MotorType.kBrushless);

    CANSparkMax rightMotor = new CANSparkMax(ArmConstants.leftMotorID, MotorType.kBrushless);

    public void updateInputs(ArmIOInputs inputs){
        armIOInputs.leftMotorCurrent = getRightMotorAmps();
        armIOInputs.rightMotorCurrent = getLeftMotorAmps();
        armIOInputs.encoderLeftPosition = getLeftMotorPosition();
        armIOInputs.encoderRightPosition = getRightMotorPosition();
    }

    private double getLeftMotorAmps() {
        return leftMotor.getOutputCurrent();
    }

    private double getRightMotorAmps() {
        return rightMotor.getOutputCurrent();
    }

    private double getLeftMotorPosition() {
        return leftMotor.getEncoder().getPosition() / ArmConstants.ticksPerFoot;
        
    }

    private double getRightMotorPosition() {
        return rightMotor.getEncoder().getPosition() / ArmConstants.ticksPerFoot;
    }

    public void setMotorPower(double leftPercent, double rightPercent) {
        rightMotor.set(-rightPercent);
        leftMotor.set(-leftPercent);
    }


}
