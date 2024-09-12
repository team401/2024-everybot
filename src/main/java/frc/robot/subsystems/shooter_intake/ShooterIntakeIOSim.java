package frc.robot.subsystems.shooter_intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIntakeIOSim implements ShooterIntakeIO {

    FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getCIM(2), 1.0, 1.0);
    PIDController controller = new PIDController(0.75, 0.1, 0.0);
    ShooterIntakeIOInputsAutoLogged shooterIntakeIOInputs;

    @Override
    public void periodic() {
        flywheelSim.update(0.02);
        if (shooterIntakeIOInputs.flywheelPowered) {
            controller.setSetpoint(shooterIntakeIOInputs.flywheelTargetSpeed);
        } else {
            controller.setSetpoint(0.0);
        }
        double calculatedVoltage = controller.calculate(getSpeed());
        calculatedVoltage = MathUtil.clamp(calculatedVoltage, -12, 12);
        flywheelSim.setInputVoltage(calculatedVoltage);
        shooterIntakeIOInputs.flywheelMotorVoltage = calculatedVoltage;
        shooterIntakeIOInputs.flywheelSpeed = flywheelSim.getAngularVelocityRPM();
        
    }

    @Override
    public void setTargetSpeed(double rpm) {
        shooterIntakeIOInputs.flywheelTargetSpeed = rpm;
    }

    @Override
    public void setFlywheelPowered(boolean powered) {
        shooterIntakeIOInputs.flywheelPowered = powered;
    }

    @Override
    public double getSpeed() {
        return flywheelSim.getAngularVelocityRPM();
    }

    @Override
    public boolean atSpeed() {
        if (shooterIntakeIOInputs.flywheelTargetSpeed > 0) {
            return (shooterIntakeIOInputs.flywheelTargetSpeed <= getSpeed());
        }
        return (shooterIntakeIOInputs.flywheelTargetSpeed >= getSpeed());
    }

    @Override
    public void setInputs(ShooterIntakeIOInputsAutoLogged shooterIntakeIOInputs) {
        this.shooterIntakeIOInputs = shooterIntakeIOInputs;
    }
}
