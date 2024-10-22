// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.shooter_intake.ShooterIntakeIOHardware;
import frc.robot.subsystems.shooter_intake.ShooterIntakeIOSim;
import frc.robot.subsystems.shooter_intake.ShooterIntakeSubsystem;
import frc.robot.subsystems.shooter_intake.ShooterIntakeSubsystem.State;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveHardwareIO;
import frc.robot.subsystems.swerve.SwerveSimIO;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotContainer {

    CANSparkMax climberMotor = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushed);

    private DoubleSupplier leftDistanceSupplier;
    private DoubleSupplier rightDistanceSupplier;
    private Supplier<Rotation2d> gyroSupplier;
    private Supplier<Pose2d> simulatedPoseSupplier;

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController masherController = new CommandXboxController(1);

    // Subsystems
    SwerveDriveSubsystem swerveDriveSubsystem;

    // Commands
    DriveWithGamepad driveWithGamepad;
    ShooterIntakeSubsystem intakeSubsystem;

    public boolean leftBumperPressed() {
        return masherController.leftBumper().getAsBoolean();
    }

    public boolean leftTriggerPressed() {
        return masherController.leftTrigger().getAsBoolean();
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        setupSubsystems();
        setupCommands();
    }

    public void setupSubsystems() {
        switch (Constants.BotConstants.botMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveHardwareIO());
                intakeSubsystem = new ShooterIntakeSubsystem(new ShooterIntakeIOHardware());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveSimIO());
                intakeSubsystem = new ShooterIntakeSubsystem(new ShooterIntakeIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveSimIO());
                intakeSubsystem = new ShooterIntakeSubsystem(new ShooterIntakeIOSim());
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

        masherController
                .b()
                .whileTrue(new InstantCommand(() -> intakeSubsystem.setTargetState(State.INTAKING)))
                .onFalse(new InstantCommand(() -> intakeSubsystem.setTargetState(State.IDLE)));
        masherController
                .y()
                .whileTrue(new InstantCommand(() -> intakeSubsystem.setTargetState(State.SHOOTING)))
                .onFalse(new InstantCommand(() -> intakeSubsystem.setTargetState(State.IDLE)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new Command() {};
    }
}
