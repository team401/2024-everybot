package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final ArmIO arm;

    private PIDController endgameController = new PIDController(0, 0, 0);

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
                endgameController.calculate(armInputs.encoderLeftPosition, endgameGoalPosition);
        double rightOutput =
                endgameController.calculate(armInputs.encoderRightPosition, endgameGoalPosition);
        arm.setMotorPower(leftOutput, rightOutput);
        double amps = armInputs.armAmps;
    }

    @Override
    public void periodic() {}
}
