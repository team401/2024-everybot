package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.parser.json.ModuleJson;
import swervelib.parser.json.MotorConfigDouble;
import swervelib.simulation.SwerveModuleSimulation;

public class SwerveSimIO implements SwerveIO {

    SwerveDriveConfiguration config;
    ChassisSpeeds truthChassisSpeeds;
    Pose2d pose;
    boolean brakeMode = false;

    // Simulated hardware.
    private final double SIM_MAX_SPEED = 4.5;
    private final int NUM_MODULES = 4;
    SimSwerveIMU simImu;
    SimSwerveMotor[] driveMotors = new SimSwerveMotor[NUM_MODULES];
    SimSwerveMotor[] angleMotors = new SimSwerveMotor[NUM_MODULES];
    SimSwerveAbsoluteEncoder[] encoders = new SimSwerveAbsoluteEncoder[NUM_MODULES];

    // Conversion factors (hardcoded for now, need to fork/update YAGSL to make this
    // better).
    private final double DRIVE_CONVERSION_FACTOR = 1.0;
    private final double ANGLE_CONVERSION_FACTOR = 1.0;
    private final MotorConfigDouble CONVERSION_FACTORS = new MotorConfigDouble(DRIVE_CONVERSION_FACTOR,
            ANGLE_CONVERSION_FACTOR);
    private final double ANGLE_OFFSET = 0.0;

    // Physical dimensions (hardcoded for now, need to fork/update YAGSL to make
    // this better).
    private final double X_METERS = 0.5;
    private final double Y_METERS = 0.5;
    private SwerveModulePhysicalCharacteristics moduleCharacteristics = new SwerveModulePhysicalCharacteristics(
            CONVERSION_FACTORS, 1.0, 1.0);

    // Controller configuration parameters (hardcoded, need to fork/update YAGsL to
    // make this better).
    private PIDFConfig velocityPidfConfig = new PIDFConfig(1.0, 0.0, 0.0, 0);
    private PIDFConfig anglePidfConfig = new PIDFConfig(1.0, 0.0, 0.0, 0.0);

    SwerveModuleConfiguration[] moduleConfigurations = new SwerveModuleConfiguration[NUM_MODULES];

    FlywheelSim turnMotor;

    public SwerveSimIO() {
        // create the simulated IMU
        simImu = new SimSwerveIMU();

        // Create simulated modules
        for (int i = 0; i < moduleConfigurations.length; i++) {
            // Initialize the module's motors and encoder.
            driveMotors[i] = new SimSwerveMotor();
            angleMotors[i] = new SimSwerveMotor();
            encoders[i] = new SimSwerveAbsoluteEncoder();

            String moduleName = "SimSwerveModule" + i;
            moduleConfigurations[i] = new SwerveModuleConfiguration(driveMotors[i], angleMotors[i], CONVERSION_FACTORS,
                    encoders[i],
                    ANGLE_OFFSET, X_METERS, Y_METERS,
                    velocityPidfConfig, anglePidfConfig, moduleCharacteristics,
                    moduleName, brakeMode);
        }

        config = new SwerveDriveConfiguration(moduleConfigurations, simImu, brakeMode,
                SwerveMath.createDriveFeedforward(
                        moduleCharacteristics.optimalVoltage,
                        SIM_MAX_SPEED,
                        moduleCharacteristics.wheelGripCoefficientOfFriction),
                moduleCharacteristics);
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    }

    @Override
    public void driveFieldOriented(ChassisSpeeds velocity) {
        truthChassisSpeeds = velocity;
    }

    @Override
    public void resetOdometry(Pose2d initialHolonomicPose) {
        pose = initialHolonomicPose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        truthChassisSpeeds = chassisSpeeds;
    }

    @Override
    public void zeroGyro() {
    }

    @Override
    public void setMotorBrake(boolean brake) {
        brakeMode = brake;
    }

    @Override
    public ChassisSpeeds getFieldVelocity() {
        return truthChassisSpeeds;
    }

    @Override
    public ChassisSpeeds getRobotVelocity() {
        return truthChassisSpeeds;
    }

    @Override
    public double getDriveBaseRadiusMeters() {
        return 0.35;
    }

    @Override
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return config;
    }
}
