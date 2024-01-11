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

        // TODO: add autolog outputs
        if(result.hasTargets()) {
            var imageCaptureTime = result.getTimestampSeconds();
            var camToTargetTransform3d = result.getBestTarget().getBestCameraToTarget();
            // cam pose based on target pose.transformBy(camToTargetTransform3d)
            
        }
    }
}
