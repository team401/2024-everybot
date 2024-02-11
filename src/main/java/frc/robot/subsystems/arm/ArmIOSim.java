package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ArmConstants;

public class ArmIOSim implements ArmIO {

    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private int leftArmEncoder;
    private int encoderIndex;
    ArmIOInputs armIOInputs = new ArmIOInputs();

    public void updateInputs(ArmIOInputsAutoLogged inputs) {

        armIOInputs.leftMotorCurrent = getLeftMotorAmps();
        armIOInputs.rightMotorCurrent = getRightMotorAmps();
        armIOInputs.encoderLeftPosition = getLeftMotorPosition();
        armIOInputs.encoderRightPosition = getRightMotorPosition();
    }

    public void setMotorPower(double leftPercent, double rightPercent) {
        // leftMotor.set(-leftPercent);
        // rightMotor.set(-rightPercent);
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
}
