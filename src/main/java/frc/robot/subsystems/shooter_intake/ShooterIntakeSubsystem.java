package frc.robot.subsystems.shooter_intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterIntakeSubsystem extends SubsystemBase {

    // Move to constants later
    private double intakeTargetRPM = -100.0;
    private double shootingTargetRPM = 300.0;
    //

    protected State currentState = State.IDLE;
    protected State targetState = State.IDLE;
    protected ShooterIntakeIO shooterIntakeIO;
    static ShooterIntakeIOInputsAutoLogged shooterIntakeIOInputs =
            new ShooterIntakeIOInputsAutoLogged();

    public ShooterIntakeSubsystem(ShooterIntakeIO shooterIntakeIO) {
        this.shooterIntakeIO = shooterIntakeIO;
        shooterIntakeIO.setInputs(shooterIntakeIOInputs);
    }

    public void periodic() {
        currentState.periodic(this);
        shooterIntakeIO.periodic();
        Logger.recordOutput("ShooterIntake.CurrentState", currentState);
        Logger.processInputs("shooterIntake", shooterIntakeIOInputs);
    }

    public void setTargetState(State target) {
        this.targetState = target;
    }

    // spotless:off
    public enum State {
        IDLE {
            @Override
            protected void periodic(ShooterIntakeSubsystem subsystem) {
                subsystem.shooterIntakeIO.setFlywheelPowered(false);
            }

            @Override
            protected void handleStateChanges(ShooterIntakeSubsystem subsystem) {
                if (subsystem.targetState == this) return;
                if (subsystem.targetState == SHOOTING){
                    if (!SHOOTING_PREP.canStart(subsystem)) return;
                    this.transitionToState(subsystem, SHOOTING_PREP);
                    return;
                }
                if (!subsystem.targetState.canStart(subsystem)) return;
                this.transitionToState(subsystem, subsystem.targetState);
            }
        },

        INTAKING {

            @Override
            protected void onStart(ShooterIntakeSubsystem subsystem) {
                subsystem.shooterIntakeIO.setFlywheelPowered(true);
            }
            
            @Override
            protected void periodic(ShooterIntakeSubsystem subsystem) {
                super.periodic(subsystem);
                subsystem.shooterIntakeIO.setTargetSpeed(subsystem.intakeTargetRPM);
            }
        },

        SHOOTING_PREP {

            @Override
            protected void onStart(ShooterIntakeSubsystem subsystem) {
                subsystem.shooterIntakeIO.setFlywheelPowered(true);
                subsystem.shooterIntakeIO.setTargetSpeed(subsystem.shootingTargetRPM);
            }

            @Override
            protected void handleStateChanges(ShooterIntakeSubsystem subsystem) {
                switch (subsystem.targetState){
                    case SHOOTING:
                        if (!SHOOTING.canStart(subsystem)) return;
                        if (!subsystem.shooterIntakeIO.atSpeed()) return;
                        this.transitionToState(subsystem, SHOOTING);
                        break;
                    default:
                        if (!IDLE.canStart(subsystem)) return;
                        this.transitionToState(subsystem, IDLE);
                }
            }

            @Override
            protected void periodic(ShooterIntakeSubsystem subsystem) {
                super.periodic(subsystem);
                subsystem.shooterIntakeIO.setTargetSpeed(subsystem.shootingTargetRPM);
            }
        },

        SHOOTING {
            @Override
            protected boolean canStart(ShooterIntakeSubsystem subsystem){
                return (subsystem.currentState == SHOOTING_PREP);
            }

            @Override
            protected void periodic(ShooterIntakeSubsystem subsystem) {
                super.periodic(subsystem);
                subsystem.shooterIntakeIO.setTargetSpeed(subsystem.shootingTargetRPM);
            }
        };

        protected void onStart(ShooterIntakeSubsystem subsystem) {}

        protected void onEnd(ShooterIntakeSubsystem subsystem) {}
        
        protected boolean canStart(ShooterIntakeSubsystem subsystem) {
            return true;
        }

        protected void transitionToState(ShooterIntakeSubsystem subsystem, State newState){
            this.onEnd(subsystem);
            subsystem.currentState = newState;
            newState.onStart(subsystem);
        }

        protected void handleStateChanges(ShooterIntakeSubsystem subsystem) {
            if (this != subsystem.targetState) {
                if (!IDLE.canStart(subsystem)) return;
                this.transitionToState(subsystem, IDLE);
            }
        }

        protected void periodic(ShooterIntakeSubsystem subsystem) {
            this.handleStateChanges(subsystem);
        }
    }
    // spotless:on

}
