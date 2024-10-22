package frc.robot.subsystems.shooter_intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterIntakeConstants.ShooterIntakeSimConstants;

public class ShooterIntakeIOSim implements ShooterIntakeIO {

    FlywheelSim flywheelSim =
            new FlywheelSim(
                    ShooterIntakeSimConstants.FLYWHEEL_DC_MOTOR,
                    ShooterIntakeSimConstants.FLYWHEEL_GEARING,
                    ShooterIntakeSimConstants.FLYWHEEL_jKg_METERS_SQUARED);
    PIDController controller =
            new PIDController(
                    ShooterIntakeSimConstants.FLYWHEEL_KP,
                    ShooterIntakeSimConstants.FLYWHEEL_KI,
                    ShooterIntakeSimConstants.FLYWHEEL_KD);
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

    @Override
    public void setVoltage(double volts) {
        // Don't do anything here.
    }
}
