package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;

public class Vision {
    public CameraIO camera;
    public Vision () {
        if(RobotBase.isReal()) {
            camera = new CameraIOReal()
        } // else in sim
    }

    public void periodic () {
        var result = camera.getResult();

        if(result.hasTargets()) {
            // change rotationSpeed (send to Drive)
        }
    }
}
