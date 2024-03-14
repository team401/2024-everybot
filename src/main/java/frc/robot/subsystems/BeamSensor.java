package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class BeamSensor extends SubsystemBase {

    DigitalInput beamBreak;

    public BeamSensor() {
        beamBreak = new DigitalInput(VisionConstants.beamBreakPort);
    }

    private int detect() {
        if (!beamBreak.get()) {
            return 1;
        }
        return 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("BeamBreak", !beamBreak.get());
        SmartDashboard.putNumber("BeamBreak 1 0", detect());
    }
}
