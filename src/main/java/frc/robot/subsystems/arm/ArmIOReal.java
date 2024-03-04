package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ArmConstants;

public class ArmIOReal implements ArmIO {

    private final CANSparkMax leftMotor =
            new CANSparkMax(ArmConstants.leftMotorID, MotorType.kBrushless);

    private final CANSparkMax rightMotor =
            new CANSparkMax(ArmConstants.leftMotorID, MotorType.kBrushless);

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.leftMotorCurrent = getRightMotorAmps();
        inputs.rightMotorCurrent = getLeftMotorAmps();

        inputs.encoderLeftPosition = getLeftMotorPosition();
        inputs.encoderRightPosition = getRightMotorPosition();

        inputs.velocity = leftMotor.getEncoder().getVelocity();
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

    public void setMotorPower(double left, double right) {
        rightMotor.setVoltage(left); // check inverted or not
        leftMotor.setVoltage(right);
    }
}
