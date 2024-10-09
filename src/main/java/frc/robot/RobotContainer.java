// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.shooter_intake.ShooterIntakeIOSim;
import frc.robot.subsystems.shooter_intake.ShooterIntakeSubsystem;
import frc.robot.subsystems.shooter_intake.ShooterIntakeSubsystem.State;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveHardwareIO;
import frc.robot.subsystems.swerve.SwerveSimIO;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotContainer {

    private DoubleSupplier leftDistanceSupplier;
    private DoubleSupplier rightDistanceSupplier;
    private Supplier<Rotation2d> gyroSupplier;
    private Supplier<Pose2d> simulatedPoseSupplier;
    ShooterIntakeSubsystem intakeSubsystem = new ShooterIntakeSubsystem(new ShooterIntakeIOSim());

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final CommandXboxController intakeController =
            new CommandXboxController(OperatorConstants.kIntakeControllerPort);

    // Subsystems
    SwerveDriveSubsystem swerveDriveSubsystem;

    // Commands
    DriveWithGamepad driveWithGamepad;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        setupSubsystems();
        setupAutonomous();
        setupCommands();
    }

    public void setupSubsystems() {
        switch (Constants.BotConstants.botMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveHardwareIO());
                // why is there no hardware for shooter? intakeSubsystem = new
                // ShooterIntakeSubsystem(new ShooterIntakeIO());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveSimIO());
                intakeSubsystem = new ShooterIntakeSubsystem(new ShooterIntakeIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveSimIO());
                break;
        }
    }

    public void setupCommands() {
        driveWithGamepad =
                new DriveWithGamepad(
                        swerveDriveSubsystem,
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> -driverController.getRightX());
        swerveDriveSubsystem.setDefaultCommand(driveWithGamepad);

        intakeController
                .b()
                .whileTrue(new InstantCommand(() -> intakeSubsystem.setTargetState(State.INTAKING)))
                .whileFalse(new InstantCommand(() -> intakeSubsystem.setTargetState(State.IDLE)));
    }

    public void autoInit() {
        swerveDriveSubsystem.getAutonomousCommand().schedule();
    }

    public void setupAutonomous() {
        NamedCommands.registerCommand(
                "IDLE",
                new InstantCommand(
                        () -> intakeSubsystem.setTargetState(ShooterIntakeSubsystem.State.IDLE)));
        NamedCommands.registerCommand(
                "INTAKING",
                new InstantCommand(
                        () ->
                                intakeSubsystem.setTargetState(
                                        ShooterIntakeSubsystem.State.INTAKING)));
        NamedCommands.registerCommand(
                "SHOOTING PREP",
                new InstantCommand(
                        () ->
                                intakeSubsystem.setTargetState(
                                        ShooterIntakeSubsystem.State.SHOOTING_PREP)));
        NamedCommands.registerCommand(
                "SHOOTING",
                new InstantCommand(
                        () ->
                                intakeSubsystem.setTargetState(
                                        ShooterIntakeSubsystem.State.SHOOTING)));
    }
}
