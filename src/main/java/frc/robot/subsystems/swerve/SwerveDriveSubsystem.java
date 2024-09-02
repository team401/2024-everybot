package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.SwerveConstants;
import swervelib.parser.SwerveDriveConfiguration;

public class SwerveDriveSubsystem extends SubsystemBase {

    private SwerveIO swerveIO;

    public SwerveDriveSubsystem(SwerveIO io) {
        swerveIO = io;
        setupPathPlanner();
    }

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto
                // has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
                // ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
                        // live in your
                        // Constants class
                        AutonConstants.TRANSLATION_PID,
                        // Translation PID constants
                        AutonConstants.ANGLE_PID,
                        // Rotation PID constants
                        SwerveConstants.MAX_SPEED,
                        // Max module speed, in m/s
                        swerveIO.getDriveBaseRadiusMeters(),
                        // Drive base radius in meters. Distance from robot center to furthest
                        // module.
                        new ReplanningConfig()
                // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent()
                            ? alliance.get() == DriverStation.Alliance.Red
                            : false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Get the path follower with events.
     *
     * @param pathName PathPlanner path name.
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String pathName) {
        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return new PathPlannerAuto(pathName);
    }

    /**
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                SwerveConstants.MAX_OTF_SPEED,
                4.0,
                SwerveConstants.MAX_OTF_OMEGA,
                Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
        // before attempting to rotate.
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveIO.drive(translation, rotation, fieldRelative);
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveIO.driveFieldOriented(velocity);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveIO.resetOdometry(initialHolonomicPose);
    }

    public Pose2d getPose() {
        return swerveIO.getPose();
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveIO.setChassisSpeeds(chassisSpeeds);
    }

    public void zeroGyro() {
        swerveIO.zeroGyro();
    }

    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing
     * forward
     *
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance() {
        if (isRedAlliance()) {
            zeroGyro();
            // Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zeroGyro();
        }
    }

    public void setMotorBrake(boolean brake) {
        swerveIO.setMotorBrake(brake);
    }

    /**
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the
     * underlying drivebase. Note, this is not the raw gyro reading, this may be
     * corrected from
     * calls to resetOdometry().
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveIO.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveIO.getRobotVelocity();
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveIO.getSwerveDriveConfiguration();
    }
}
