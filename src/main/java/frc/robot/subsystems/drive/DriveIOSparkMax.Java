package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.DriveConstants;

public class DriveIOSparkMax implements DriveIO {
    private final CANSparkMax frontLeft =
            new CANSparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);
    private final CANSparkMax backLeft =
            new CANSparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
    private final CANSparkMax frontRight =
            new CANSparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
    private final CANSparkMax backRight =
            new CANSparkMax(DriveConstants.backRightID, MotorType.kBrushless);

    public DriveIOSparkMax() {
        frontLeft.setSmartCurrentLimit(40);
        backLeft.setSmartCurrentLimit(40);
        frontRight.setSmartCurrentLimit(40);
        backRight.setSmartCurrentLimit(40);

        frontRight.setInverted(true);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftAppliedVolts = frontLeft.getBusVoltage() * frontLeft.getAppliedOutput();
        inputs.leftCurrentAmps =
                new double[] {frontLeft.getOutputCurrent(), backLeft.getOutputCurrent()};

        inputs.rightAppliedVolts = frontRight.getBusVoltage() * frontRight.getAppliedOutput();
        inputs.rightCurrentAmps =
                new double[] {frontRight.getOutputCurrent(), backRight.getOutputCurrent()};
    }

    @Override
    public void setVoltage(double leftVoltage, double rightVoltage) {
        frontLeft.setVoltage(leftVoltage);
        frontRight.setVoltage(rightVoltage);
    }
}
