package frc.robot.subsystems.vision;

import java.io.IOException;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;

public class VisionSimulation {
    private VisionSystemSim sim;
    private TargetModel targetModel;
    private AprilTagFieldLayout tagLayout;

    public VisionSimulation () {
        // create simulated field
        sim = new VisionSystemSim(Constants.VisionConstants.CAMERA_NAME);
        targetModel = TargetModel.kAprilTag36h11;
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            System.out.println(e); // quit program ?
        }
        sim.addAprilTags(tagLayout);

        // create simulated camera
    }
}
