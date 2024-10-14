package frc.robot.subsystems.shooter_intake;

import static frc.robot.Constants.ShooterIntakeConstants.ShooterIntakeSimConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ShooterIntakeConstants.ShooterIntakeSimConstants;

public class ShooterIntakeIOHardware implements ShooterIntakeIO {

    private CANSparkMax leftIntake =
            new CANSparkMax(ShooterIntakeHardwareConstants.leftIntakeMotorID, MotorType.kBrushless);
    private CANSparkMax rightIntake =
            new CANSparkMax(
                    ShooterIntakeHardwareConstants.rightIntakeMotorID, MotorType.kBrushless);
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

        // flywheelSim.setInputVoltage(calculatedVoltage);

        shooterIntakeIOInputs.flywheelMotorVoltage = calculatedVoltage;
        shooterIntakeIOInputs.flywheelSpeed = flywheelSim.getAngularVelocityRPM();
    }

    @Override
    public double getSpeed() {
        return flywheelSim.getAngularVelocityRPM();
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
