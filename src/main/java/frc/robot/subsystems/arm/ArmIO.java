package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {

        public double leftMotorCurrent = 0.0;
        public double rightMotorCurrent = 0.0;
        public double encoderLeftPosition = 0.0;
        public double encoderRightPosition = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setMotorPower(double left, double right) {}
}
