package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final ArmIO arm;

    private PIDController armController = new PIDController(0, 0, 0);

    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private double leftEndgameMotorPower = 0.5;
    private double rightEndgameMotorPower = 0.5;
    private double endgameGoalPosition;

    public Arm(ArmIO arm) {
        this.arm = arm;
    }

    public void armDown() {
        endgameGoalPosition = Constants.ArmConstants.armUp;
    }

    public void armUp() {
        endgameGoalPosition = Constants.ArmConstants.armDown;
    }

    public void armControl() {
        double leftOutput =
                armController.calculate(armInputs.encoderLeftPosition, endgameGoalPosition);
        double rightOutput =
                armController.calculate(armInputs.encoderRightPosition, endgameGoalPosition);
        arm.setMotorPower(leftOutput, rightOutput);
        double amps = armInputs.armAmps;
    }

    @Override
    public void periodic() {
        arm.updateInputs(armInputs);
        armController.setPID(0, 0, 0);
        armControl();

        Logger.recordOutput(
                "endgame/Elevator3d",
                new Pose3d(
                        0.0,
                        0.0,
                        armInputs.encoderLeftPosition + 0.1,
                        new Rotation3d(0, 0, 0)));
    }
}
