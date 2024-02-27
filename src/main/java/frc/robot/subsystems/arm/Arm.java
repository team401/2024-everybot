package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.armControl;

import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.DialogOwner;

import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

    private final ArmIO arm;

    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private armControl mode;

    private PIDController rotationController;

    private double leftMotorPower;
    private double rightMotorPower;

    private DoubleSupplier currentPosition;
    private DoubleSupplier goalPosition;

    public Arm(ArmIO arm) {
        this.arm = arm;

        rotationController =
                new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void armControl(armControl mode) {
        this.mode = mode;

        this.leftMotorPower = rotationController.calculate(currentPosition.getAsDouble(), goalPosition.getAsDouble());
        this.rightMotorPower = rotationController.calculate(currentPosition.getAsDouble(), goalPosition.getAsDouble());

        switch (mode) {
            case UP:
                armSetVolts(leftMotorPower, rightMotorPower);
            case DOWN:
                armSetVolts(leftMotorPower, rightMotorPower);
            case STOP:
                armSetVolts(leftMotorPower, rightMotorPower);
            default: 
                armSetVolts(leftMotorPower, rightMotorPower);
        }
    }

    public void armSetVolts(double left, double right){
        arm.setMotorPower(left, right);
    }

    public void setCurrentPosition(){
        this.currentPosition = () -> {return armInputs.encoderLeftPosition;};
    }

    public void setGoalPosition(DoubleSupplier goalPosition){
        this.goalPosition = goalPosition;
    }



    @Override
    public void periodic() {
        arm.updateInputs(armInputs);
        armControl(mode);

        Logger.recordOutput(
                "arm/Elevator3d",
                new Pose3d(0.0, 0.0, armInputs.encoderLeftPosition + 0.1, new Rotation3d(0, 0, 0)));
        
        setCurrentPosition();
    }
}
