// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.IOException;
import java.util.Collections;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static enum DriveTrainState {
        /*Manual control */
        MANUAL,

        /*Aiming to shot */
        AIM,

        /*Auto pathfollowing */
        PATHFOLLOW,
    }

    public static final int pigeonID = 0; // placeholder

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static enum iMode {
        /* Joystick Imputs */
        Stick,

        /* Keyboard Inputs */
        Key,
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class BotConstants {
        public static Mode botMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
        public static iMode inputMode =
                iMode.Stick; // my puny brain can't think of how to set inputMode automatically
    }

    public static class VisionConstants { // ALL PLACEHOLDERS
        public static final String CAMERA_NAME = "";
        public static final double CAMERA_HEIGHT_METERS = 0.7;
        public static final double TARGET_HEIGHT_METERS = 0.8; // changes per goal
        public static final double CAMERA_PITCH_RADIANS =
                0; // difference betweeen horizontal and camera angle
        public static final int CAMERA_FPS = 20;
        public static final double IDEAL_GOAL_RANGE_METERS = 1;

        // simulation
        public static final int RESOLUTION_WIDTH = 480;
        public static final int RESOLUTION_HEIGHT = 480;
        public static final double CAM_DIAG_FOV = 140.0;
        public static final double MAX_LED_RANGE = 0.0;
        public static final double MIN_TARGET_AREA = 0.0;
        public static final Translation3d BOT_TO_CAM_TRL =
                new Translation3d(
                        0.1, 0, 0.5); // 0.1 from robot pose forward, 0.5 meters up from robot pose
        public static final Rotation3d BOT_TO_CAMERA_ROT =
                new Rotation3d(0, Math.toRadians(-15), 0);
        public static final boolean SIM_FIELD_ENABLED = false;
    }

    public static class FieldConstants {
        public static final AprilTagFieldLayout FIELD_LAYOUT = initLayout();
    }

    private static AprilTagFieldLayout initLayout() {
        AprilTagFieldLayout layout;
        try {
            layout =
                    AprilTagFieldLayout.loadFromResource(
                            AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException ioe) {
            layout = new AprilTagFieldLayout(Collections.emptyList(), 0.0, 0.0);
        }
        return layout;
    }

    public static class DriveConstants {
        public static final double TRACK_WIDTH = 0.0; // placeholder
        public static final int frontLeftID = 0; // placeholder
        public static final int backLeftID = 0; // placeholder
        public static final int frontRightID = 0; // placeholder
        public static final int backRightID = 0; // placeholder
        public static final int pigeonID = 0; // placeholder

        public static final double GEAR_RATIO = 10.0;

        public static final double autoDrivePercent = 0.0; // placeholder

        public static final double WHEEL_RADIUS = 1.0; // placeholder

        public static final double kS = 0.1; // placeholder
        public static final double kP = 0.1; // placeholder
        public static final double kD = 0.1; // placeholder
        public static final double kV = 0.1; // placeholder
    }

    public static class SimConstants {
        public static final double KP = 0.2; // placeholder
        public static final double KD = 0.0; // placeholder
    }
}
