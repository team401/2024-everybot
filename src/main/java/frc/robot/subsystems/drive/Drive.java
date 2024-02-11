package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.driveTrainState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    
    private double forward;
    private double rotation;
    private double vx, vy, omega = 0.0;
    private final DriveIO io;
    private driveTrainState mode;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final SimpleMotorFeedforward driveff =
            new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV);
    ChassisSpeeds speed =
            ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, Math.PI / 2.0, this.getGyroRotation2d());
    Pose2d pose = Pose2d(vx, vy, omega);
    DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(Units.inchesToMeters(27.0));

    RobotContainer robotContainer = new RobotContainer();

    public Drive(DriveIO io) {
        this.io = io;
        configurePathPlanner();
    }

    private Pose2d Pose2d(double vx2, double vy2, double omega2) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'Pose2d'");
    }

    public PathPlannerPath followPathCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return path;
    }

    public void configurePathPlanner() {
        AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a
                // starting pose)
                this::getCurrentSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the
                // options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
                );
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d pose) {
        vx = 0;
        vy = 0;
        omega = 0;
        pose = Pose2d(0, 0, 0);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return this.speed;
    }

    public void drive(ChassisSpeeds speed) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speed);
        driveArcade(wheelSpeeds.leftMetersPerSecond, omega);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
        controlDriveTrain();
    }

    public void setArcadeDrive(double forward, double rotation, driveTrainState mode) {
        this.forward = forward;
        this.rotation = rotation;
        this.mode = mode;
    }

    /** Run open loop based on stick positions. */
    public void driveArcade(double xSpeed, double zRotation) {
        var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
        io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
    }

    /** Stops the drive. */
    public void stop() {
        io.setVoltage(0.0, 0.0);
    }

    public Rotation2d getGyroRotation2d() {
        return inputs.gyroYaw;
    }

    /** Returns the position of the left wheels in meters. */
    @AutoLogOutput
    public double getLeftPositionMeters() {
        return inputs.leftPositionRad * Constants.DriveConstants.WHEEL_RADIUS;
    }

    /** Returns the position of the right wheels in meters. */
    @AutoLogOutput
    public double getRightPositionMeters() {
        return inputs.rightPositionRad * Constants.DriveConstants.WHEEL_RADIUS;
    }

    /** Returns the velocity of the left wheels in meters/second. */
    @AutoLogOutput
    public double getLeftVelocityMetersPerSec() {
        return inputs.leftVelocityRadPerSec * Constants.DriveConstants.WHEEL_RADIUS;
    }

    /** Returns the velocity of the right wheels in meters/second. */
    @AutoLogOutput
    public double getRightVelocityMetersPerSec() {
        return inputs.rightVelocityRadPerSec * Constants.DriveConstants.WHEEL_RADIUS;
    }

    public Pose2d getSimulatedPose() {
        return inputs.simulatedPose;
    }

    public void updateMode(driveTrainState mode) {
        this.mode = mode;
    }

    public void controlDriveTrain() {
        switch (mode) {
            case MANUAL:
                this.driveArcade(forward, rotation);
                break;
            case AIM:
                this.stop();
                break;
            case AUTO:
                robotContainer.setAutonomousCommand("Center 3pc");
            default:
                this.stop();
                break;
        }
    }
}
