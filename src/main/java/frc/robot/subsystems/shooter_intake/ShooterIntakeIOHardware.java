package frc.robot.subsystems.shooter_intake;

import static frc.robot.Constants.ShooterIntakeConstants.ShooterIntakeSimConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIntakeIOHardware implements ShooterIntakeIO {

    TalonFX flywheel = new TalonFX(1);
    PIDController controller =
            new PIDController(
                    ShooterIntakeSimConstants.FLYWHEEL_KP,
                    ShooterIntakeSimConstants.FLYWHEEL_KI,
                    ShooterIntakeSimConstants.FLYWHEEL_KD);
    ShooterIntakeIOInputsAutoLogged shooterIntakeIOInputs;

    @Override
    public void periodic() {
        if (shooterIntakeIOInputs.flywheelPowered) {
            controller.setSetpoint(shooterIntakeIOInputs.flywheelTargetSpeed);
        } else {
            controller.setSetpoint(0.0);
        }
        double calculatedVoltage = controller.calculate(getSpeed());
        calculatedVoltage = MathUtil.clamp(calculatedVoltage, -12, 12);
        flywheel.setVoltage(calculatedVoltage);
        shooterIntakeIOInputs.flywheelMotorVoltage = calculatedVoltage;
        shooterIntakeIOInputs.flywheelSpeed = flywheel.getVelocity().getValueAsDouble(); //unit conversions
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
        return flywheel.getVelocity().getValueAsDouble(); //todo unit conversions
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
