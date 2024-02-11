package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

    private final ArmIO arm;

    private PIDController armController = new PIDController(0, 0, 0);

    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private double leftMotorPowewr;
    private double rightMotorPower;
    private double endgameGoalPosition;
    private boolean upDown;

    public Arm(ArmIO arm) {
        this.arm = arm;
    }

    public void armDown() {
        endgameGoalPosition = Constants.ArmConstants.armDown;
    }

    public void armUp() {
        endgameGoalPosition = Constants.ArmConstants.armUp;
    }

    public void armControl(boolean upDown) {
        this.upDown = upDown;
        if (upDown) {
            armUp();
        } else {
            armDown();
        }
        leftMotorPowewr =
                armController.calculate(armInputs.encoderLeftPosition, endgameGoalPosition);
        rightMotorPower =
                armController.calculate(armInputs.encoderRightPosition, endgameGoalPosition);
        arm.setMotorPower(leftMotorPowewr, rightMotorPower);
    }

    @Override
    public void periodic() {
        arm.updateInputs(armInputs);
        armController.setPID(0, 0, 0);
        armControl(upDown);

        Logger.recordOutput(
                "arm/Elevator3d",
                new Pose3d(0.0, 0.0, armInputs.encoderLeftPosition + 0.1, new Rotation3d(0, 0, 0)));
    }
}
