// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOTalonFX;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotContainer {
    private Drive drive;

    // init navigation
    private Navigation nav;

    private DoubleSupplier leftDistanceSupplier;
    private DoubleSupplier rightDistanceSupplier;
    private Supplier<Rotation2d> gyroSupplier;
    private Supplier<Pose2d> simulatedPoseSupplier;

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private Joystick leftJoystick = new Joystick(0);
    private Joystick rightJoystick = new Joystick(1);

    private final SendableChooser<Command> autoChooser;

    // private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        switch (Constants.BotConstants.botMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(new DriveIOTalonFX()); // Spark Max/Spark Flex + brushed, no
                // encoders
                // drive = new Drive(new DriveIOSparkMax()); // Spark Max/Spark Flex + NEO/Vortex
                // drive = new Drive(new DriveIOTalonSRX()); // Talon SRX + brushed, no encoders
                // drive = new Drive(new DriveIOTalonFX()); // Talon FX (Falon 500/Kraken X60)
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(new DriveIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(new DriveIO() {});
                break;
        }
        // make suppliers for navigation
        leftDistanceSupplier =
                () -> {
                    return drive.getLeftPositionMeters();
                };
        rightDistanceSupplier =
                () -> {
                    return drive.getRightPositionMeters();
                };
        gyroSupplier =
                () -> {
                    return drive.getGyroRotation2d();
                };
        simulatedPoseSupplier =
                () -> {
                    return drive.getSimulatedPose();
                };
        nav =
                new Navigation(
                        leftDistanceSupplier,
                        rightDistanceSupplier,
                        gyroSupplier,
                        simulatedPoseSupplier);

        // Set up auto routines

        // Set up feedforward characterization
        // //autoChooser.addOption(
        //     "Drive FF Characterization",
        //     new FeedForward(
        //         drive, (volts) -> drive.driveVolts(volts, volts),
        // drive::getCharacterizationVelocity));

        // Configure the button bindings
        // configureButtonBindings();
        // Configure the trigger bindings

        drive.setDefaultCommand(
                new ArcadeDrive(
                        drive,
                        () -> -leftJoystick.getRawAxis(1),
                        () -> rightJoystick.getRawAxis(0)));

        configureBindings();
        SmartDashboard.putData("Auto Choose", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drive.setDefaultCommand(
                new ArcadeDrive(
                        drive,
                        () -> driverController.getLeftY(),
                        () -> driverController.getRightY()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command setAutonomousCommand(String path) {
        return new PathPlannerAuto(path);
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
