package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {

    private final ElevatorSim elevatorSim =
            new ElevatorSim(DCMotor.getNeoVortex(2), 20, 1.814, 0.02231009, 0.0, 0.45, true, 0.0);

    double appliedVolts = 0.0;

    @Override
    public void setMotorPower(double left, double right) {
        appliedVolts = left;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        elevatorSim.update(Constants.loopTime);
        elevatorSim.setInputVoltage(appliedVolts);

        inputs.leftMotorCurrent = appliedVolts;
        inputs.rightMotorCurrent = appliedVolts;

        inputs.encoderLeftPosition = elevatorSim.getPositionMeters();
        inputs.encoderRightPosition = elevatorSim.getPositionMeters();
    }
}
