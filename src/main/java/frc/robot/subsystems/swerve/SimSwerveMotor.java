package frc.robot.subsystems.swerve;

import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.motors.SwerveMotor;
import swervelib.parser.PIDFConfig;

public class SimSwerveMotor extends SwerveMotor {

    public SimSwerveMotor() {
    }

    @Override
    public void factoryDefaults() {
    }

    @Override
    public void clearStickyFaults() {
    }

    @Override
    public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder) {
        return this;
    }

    @Override
    public void configureIntegratedEncoder(double positionConversionFactor) {
    }

    @Override
    public void configurePIDF(PIDFConfig config) {
    }

    @Override
    public void configurePIDWrapping(double minInput, double maxInput) {
    }

    @Override
    public void setMotorBrake(boolean isBrakeMode) {
    }

    @Override
    public void setInverted(boolean inverted) {
    }

    @Override
    public void burnFlash() {
    }

    @Override
    public void set(double percentOutput) {
    }

    @Override
    public void setReference(double setpoint, double feedforward) {
    }

    @Override
    public void setReference(double setpoint, double feedforward, double position) {
    }

    @Override
    public double getVoltage() {
    }

    @Override
    public void setVoltage(double voltage) {
    }

    @Override
    public double getAppliedOutput() {
    }

    @Override
    public double getVelocity() {
    }

    @Override
    public double getPosition() {
    }

    @Override
    public void setPosition(double position) {
    }

    @Override
    public void setVoltageCompensation(double nominalVoltage) {

    }

    @Override
    public void setCurrentLimit(int currentLimit) {
    }

    @Override
    public void setLoopRampRate(double rampRate) {
    }

    @Override
    public Object getMotor() {
    }

    @Override
    public boolean isAttachedAbsoluteEncoder() {
    }

}
