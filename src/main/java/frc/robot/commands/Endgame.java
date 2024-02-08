package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class Endgame extends Command {
    private final Arm armSub;

    private final double rightEndgameMotorPower;
    private final double leftEndgameMotorPower;

    public Endgame(Arm armSub, double right, double left) {
        this.armSub = armSub;
        rightEndgameMotorPower = right;
        leftEndgameMotorPower = left;
    }
}
