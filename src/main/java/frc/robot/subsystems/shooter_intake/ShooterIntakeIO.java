package frc.robot.subsystems.shooter_intake;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIntakeIO {

    @AutoLog
    public static class ShooterIntakeIOInputs {
        public double flywheelTargetSpeed = 0.0;
        public double flywheelSpeed = 0.0;
        public double flywheelMotorVoltage = 0.0;
        public boolean flywheelPowered = false;
    }

    public void periodic();

    public void setTargetSpeed(double rpm);

    public void setFlywheelPowered(boolean powered);

    public double getSpeed();

    public boolean atSpeed();

    public void setInputs(ShooterIntakeIOInputsAutoLogged shooterIntakeIOInputs);
}
