package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class ToFSensor extends SubsystemBase {

    private TimeOfFlight sensor;
    private double distmin;
    private double distmax;
    private double lastdist = 0;
    private int counter = 0;

    public ToFSensor() {
        sensor = new TimeOfFlight(VisionConstants.sensorID);
        distmin = VisionConstants.minDistSensor;
        distmax = VisionConstants.maxDistSensor;
        sensor.setRangingMode(RangingMode.Short, 24.0); // Maybe 40 so it's every other run
        sensor.setRangeOfInterest(8, 8, 12, 12); // Narrower FOV
    }

    public boolean objectDetected() {
        double dist = getDistance();
        if (getDistance() == -1) {
            return false;
        }
        if ((dist > distmin) && (dist < distmax)) {
            return true;
        }
        return false;
    }

    public double getDistance() {
        double dist = sensor.getRange();
        if (!sensor.isRangeValid()) {
            if (counter < 10) {
                return lastdist;
            }
            counter++;
            return -1;
        }
        lastdist = dist;
        counter = 0;
        return dist;
    }

    @Override
    public void periodic() {}
}
