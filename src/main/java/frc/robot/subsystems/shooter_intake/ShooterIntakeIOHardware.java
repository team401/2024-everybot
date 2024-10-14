package frc.robot.subsystems.shooter_intake;

import static frc.robot.Constants.ShooterIntakeConstants.ShooterIntakeSimConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ShooterIntakeConstants.ShooterIntakeHardwareConstants;
import frc.robot.Constants.ShooterIntakeConstants.ShooterIntakeSimConstants;

public class ShooterIntakeIOHardware implements ShooterIntakeIO {

    private CANSparkMax leftIntake =
            new CANSparkMax(ShooterIntakeHardwareConstants.topIntakeMotorID, MotorType.kBrushless);
    private CANSparkMax rightIntake =
            new CANSparkMax(
                    ShooterIntakeHardwareConstants.bottomIntakeMotorID, MotorType.kBrushless);
    PIDController leftController =
            new PIDController(
                    ShooterIntakeSimConstants.FLYWHEEL_KP,
                    ShooterIntakeSimConstants.FLYWHEEL_KI,
                    ShooterIntakeSimConstants.FLYWHEEL_KD);
    PIDController rightController =
            new PIDController(
                    ShooterIntakeSimConstants.FLYWHEEL_KP,
                    ShooterIntakeSimConstants.FLYWHEEL_KI,
                    ShooterIntakeSimConstants.FLYWHEEL_KD);

    ShooterIntakeIOInputsAutoLogged shooterIntakeIOInputs;

    @Override
    public void periodic() {
        if (shooterIntakeIOInputs.flywheelPowered) {
            leftController.setSetpoint(shooterIntakeIOInputs.flywheelTargetSpeed);
        } else {
            leftController.setSetpoint(0.0);
        }
        if (shooterIntakeIOInputs.flywheelPowered) {
            rightController.setSetpoint(shooterIntakeIOInputs.flywheelTargetSpeed);
        } else {
            rightController.setSetpoint(0.0);
        }
        double calculatedLeftVoltage =
                leftController.calculate(leftIntake.getEncoder().getVelocity());
        double calculatedRightVoltage =
                rightController.calculate(rightIntake.getEncoder().getVelocity());

        calculatedLeftVoltage = MathUtil.clamp(calculatedLeftVoltage, -12, 12);
        calculatedLeftVoltage = MathUtil.clamp(calculatedRightVoltage, -12, 12);

        leftIntake.set(calculatedLeftVoltage / 12);
        rightIntake.set(calculatedRightVoltage / 12);
        // flywheelSim.setInputVoltage(calculatedVoltage);

        shooterIntakeIOInputs.flywheelMotorVoltage = calculatedLeftVoltage;
        shooterIntakeIOInputs.flywheelSpeed = getSpeed();
        // shooterIntakeIOInputs.flywheelSpeed = flywheelSim.getAngularVelocityRPM();
    }

    @Override
    public double getSpeed() {
        // return flywheelSim.getAngularVelocityRPM();
        return leftIntake.getEncoder().getVelocity();
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
