package frc.robot.subsystems.swerve;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.imu.SwerveIMU;

public class SimSwerveIMU extends SwerveIMU {

    @Override
    public void factoryDefault() {
    }

    @Override
    public void clearStickyFaults() {
    }

    @Override
    public void setOffset(Rotation3d offset) {
    }

    @Override
    public void setInverted(boolean invertIMU) {
    }

    @Override
    public Rotation3d getRawRotation3d() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRawRotation3d'");
    }

    @Override
    public Rotation3d getRotation3d() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRotation3d'");
    }

    @Override
    public Optional<Translation3d> getAccel() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAccel'");
    }

    @Override
    public double getRate() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRate'");
    }

    @Override
    public Object getIMU() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getIMU'");
    }

}
