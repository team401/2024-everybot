package frc.robot.subsystems.shooter_intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ShooterIntakeConstants.ShooterIntakeSimConstants;
import frc.robot.constants.ShooterIntakeConstants.ShooterIntakeHardwareConstants;

public class ShooterIntakeIOHardware implements ShooterIntakeIO {

    private CANSparkMax leftIntake =
            new CANSparkMax(ShooterIntakeHardwareConstants.topIntakeMotorID, MotorType.kBrushless);
    private CANSparkMax rightIntake =
            new CANSparkMax(
                    ShooterIntakeHardwareConstants.bottomIntakeMotorID, MotorType.kBrushless);
    private double targetVoltage = 0.0;

    PIDController leftController =
            new PIDController(
                    ShooterIntakeSimConstants.FLYWHEEL_KP,
                    ShooterIntakeSimConstants.FLYWHEEL_KI,
                    ShooterIntakeSimConstants.FLYWHEEL_KD);

    ShooterIntakeIOInputsAutoLogged shooterIntakeIOInputs;

    @Override
    public void periodic() {
        rightIntake.setInverted(true);
        leftIntake.setIdleMode(CANSparkBase.IdleMode.kBrake);

        // if (shooterIntakeIOInputs.flywheelPowered) {
        //     leftController.setSetpoint(shooterIntakeIOInputs.flywheelTargetSpeed);
        // } else {
        //     leftController.setSetpoint(0.0);
        // }
        // double calculatedLeftVoltage =
        //         leftController.calculate(leftIntake.getEncoder().getVelocity());

        // calculatedLeftVoltage = MathUtil.clamp(calculatedLeftVoltage, -12, 12);
        // Override PID and just set to max power

        if (shooterIntakeIOInputs.flywheelPowered) {
            leftIntake.set(targetVoltage / 12);
            rightIntake.set(targetVoltage / 12);
        } else {
            leftIntake.set(0.0);
            rightIntake.set(0.0);
        }
        // flywheelSim.setInputVoltage(calculatedVoltage);

        shooterIntakeIOInputs.flywheelMotorVoltage = targetVoltage;
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
    public void setVoltage(double volts) {
        targetVoltage = volts;
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
