// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.shooter_intake.ShooterIntakeIO;
import frc.robot.subsystems.shooter_intake.ShooterIntakeSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RobotContainer {

    private DoubleSupplier leftDistanceSupplier;
    private DoubleSupplier rightDistanceSupplier;
    private Supplier<Rotation2d> gyroSupplier;
    private Supplier<Pose2d> simulatedPoseSupplier;
    private ShooterIntakeSubsystem intakeSubsystem =
            new ShooterIntakeSubsystem(
                    new ShooterIntakeIO() {

                        @Override
                        public void setVoltage(double volts) {
                            // TODO Auto-generated method stub
                            throw new UnsupportedOperationException(
                                    "Unimplemented method 'setVoltage'");
                        }
                    });

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private Joystick leftJoystick = new Joystick(0);
    private Joystick rightJoystick = new Joystick(1);

    // private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.BotConstants.botMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations

                break;

            default:
                // Replayed robot, disable IO implementations

                break;
        }
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
