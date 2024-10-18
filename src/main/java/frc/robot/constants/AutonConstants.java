package frc.robot.constants;

import com.pathplanner.lib.util.PIDConstants;

public final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID =
            new PIDConstants(15, 0, 0); // P originally = 0.7
    public static final PIDConstants ANGLE_PID = new PIDConstants(1, 0, 0.01); // P originally = 0.4
}
