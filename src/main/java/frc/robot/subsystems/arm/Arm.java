package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

    private final ArmIO arm;

    private final ArmIOInputsAutoLogged armInputs = new ArmIOInputsAutoLogged();

    private boolean upDown;

    public Arm(ArmIO arm) {
        this.arm = arm;
    }

    public void armControl(boolean upDown) {
        this.upDown = upDown;
        if (upDown) {
            arm.setMotorPower(4.0, 4.0);
        } else {
            arm.setMotorPower(-4.0, -4.0);
        }
    }

    @Override
    public void periodic() {
        arm.updateInputs(armInputs);
        armControl(upDown);

        Logger.recordOutput(
                "arm/Elevator3d",
                new Pose3d(0.0, 0.0, armInputs.encoderLeftPosition + 0.1, new Rotation3d(0, 0, 0)));
    }
}
