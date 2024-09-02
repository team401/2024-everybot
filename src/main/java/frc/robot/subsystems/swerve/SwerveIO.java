package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveIO {

    public static class SwerveInputs {}

    public void updateInputs(SwerveInputs inputs);

    public void updateOutputs();

    /**
     * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a
     * rotation rate, and calculates and commands module states accordingly. Can use either
     * open-loop or closed-loop velocity control for the wheel velocities. Also has field- and
     * robot-relative modes, which affect how the translation vector is used.
     *
     * @param translation {@link Translation2d} that is the commanded linear velocity of the robot,
     *     in meters per second. In robot-relative mode, positive x is torwards the bow (front) and
     *     positive y is torwards port (left). In field-relative mode, positive x is away from the
     *     alliance wall (field North) and positive y is torwards the left wall when looking through
     *     the driver station glass (field West).
     * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
     *     field/robot relativity.
     * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative);

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity);

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset
     * when calling this method. However, if either gyro angle or module position is reset, this
     * must be called in order for odometry to keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose);

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose();

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds);

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro();

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake);

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity();

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity();

    /**
     * Gets the drive base radius in meters.
     *
     * @return The robot's drive base radius in meters.
     */
    public double getDriveBaseRadiusMeters();
}
