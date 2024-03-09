package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import frc.robot.Constants.VisionConstants;

public class ToFSensor {

    private TimeOfFlight sensor;
    private double distmin;
    private double distmax;

    public ToFSensor() {
        sensor = new TimeOfFlight(VisionConstants.sensorID);
        distmin = VisionConstants.minDistSensor;
        distmax = VisionConstants.maxDistSensor;
        sensor.setRangingMode(RangingMode.Short, 24.0);
    }

    public boolean objectDetected() {
        double dist = sensor.getRange();
        while (!sensor.isRangeValid()) {
            dist = sensor.getRange();
        }
        if ((dist > distmin) && (dist < distmax)) {
            return true;
        }
        return false;
    }

    public double getDistance() {
        double dist = sensor.getRange();
        while (!sensor.isRangeValid()) {
            dist = sensor.getRange();
        }
        return dist;
    }
}
