package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import frc.robot.Constants.DriveConstants;

public class ToFSenso{

  TimeOfFlight sensor;
  int distmin;
  int distmax;

  public ToFSensor(){
    sensor = new TimeOfFlight(DriveConstants.sensorID);
    distmin = DriveConstants.minDistSensor;
    distmax = DriveConstants.maxDistSensor;
		sesnor.setRangingMode(RangingMode.Short, 24.0)
  }

  public boolean objectDetected(){
    double dist = sensor.getRange();
		while(!sensor.isRangeValid){
			dist = sensor.getRange();
		}
		if((dist > distmin) && (dist < distmax)){
			return true;
		}
		return false;
	}
}

	
	

  

  
  