// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveHardwareIO;
import frc.robot.subsystems.swerve.SwerveSimIO;

public class RobotContainer {

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // Subsystems
    SwerveDriveSubsystem swerveDriveSubsystem;

    // Commands
    DriveWithGamepad driveWithGamepad;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        setupSubsystems();
    }

    public void setupSubsystems() {
        switch (Constants.BotConstants.botMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveHardwareIO());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                swerveDriveSubsystem = new SwerveDriveSubsystem(new SwerveSimIO());
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
                        () -> driverController.getLeftX(),
                        () -> driverController.getLeftY(),
                        () -> driverController.getRightY());
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
