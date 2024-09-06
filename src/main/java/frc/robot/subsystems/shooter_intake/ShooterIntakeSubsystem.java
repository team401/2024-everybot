package frc.robot.subsystems.shooter_intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterIntakeSubsystem extends SubsystemBase {

    // Move to constants later
    double intakeTargetRPM = -100.0;
    double shootingTargetRPM = 300.0;
    //

    protected State currentState = State.IDLE;
    protected State targetState = State.IDLE;
    ShooterIntakeIO io;
    static ShooterIntakeIOInputsAutoLogged shooterIntakeIOInputs =
            new ShooterIntakeIOInputsAutoLogged();

    public ShooterIntakeSubsystem(ShooterIntakeIO io) {
        this.io = io;
        io.setInputs(shooterIntakeIOInputs);
    }

    public void periodic() {
        currentState.periodic(this);
        io.periodic();
        Logger.recordOutput("Shooter.flyWheel.CurrentState", currentState);
        Logger.recordOutput("Shooter.flyWheel.speed", io.getSpeed());
    }

    public void setTargetState(State target) {
        this.targetState = target;
    }

    // spotless:off
    public enum State {
        IDLE {

            @Override
            void handleStateChanges(ShooterIntakeSubsystem subsystem) {
                if (subsystem.targetState == this) return;
                switch (subsystem.targetState){
                    case SHOOTING:
                        if (!SHOOTING_PREP.canStart(subsystem)) return;
                        this.transitionToState(subsystem, SHOOTING_PREP);
                        break;
                    default:
                        if (!subsystem.targetState.canStart(subsystem)) return;
                        this.transitionToState(subsystem, subsystem.targetState);
                }
            }

            @Override
            void onStart(ShooterIntakeSubsystem subsystem) {
                subsystem.io.setFlywheelPowered(false);
            }
        },

        INTAKING {

            @Override
            void onStart(ShooterIntakeSubsystem subsystem) {
                subsystem.io.setFlywheelPowered(true);
            }
            
            @Override
            void periodic(ShooterIntakeSubsystem subsystem) {
                super.periodic(subsystem);
                subsystem.io.setTargetSpeed(subsystem.intakeTargetRPM);
            }
        },

        SHOOTING_PREP {

            @Override
            void onStart(ShooterIntakeSubsystem subsystem) {
                subsystem.io.setFlywheelPowered(true);
                subsystem.io.setTargetSpeed(subsystem.shootingTargetRPM);
            }

            @Override
            void handleStateChanges(ShooterIntakeSubsystem subsystem) {
                switch (subsystem.targetState){
                    case SHOOTING:
                        if (!SHOOTING.canStart(subsystem)) return;
                        if (!subsystem.io.atSpeed()) return;
                        this.transitionToState(subsystem, SHOOTING);
                        break;
                    default:
                        if (!subsystem.targetState.canStart(subsystem)) return;

                        this.transitionToState(subsystem, subsystem.targetState);
                }
            }

            @Override
            void periodic(ShooterIntakeSubsystem subsystem) {
                super.periodic(subsystem);
                subsystem.io.setTargetSpeed(subsystem.shootingTargetRPM);
            }
        },

        SHOOTING {

            @Override
            void periodic(ShooterIntakeSubsystem subsystem) {
                super.periodic(subsystem);
                subsystem.io.setTargetSpeed(subsystem.shootingTargetRPM);
            }
            
            @Override
            boolean canStart(ShooterIntakeSubsystem subsystem){
                return (subsystem.currentState == SHOOTING_PREP);
            }
        };

        void periodic(ShooterIntakeSubsystem subsystem) {
            this.handleStateChanges(subsystem);
            Logger.processInputs("shooterintake", shooterIntakeIOInputs);
        }

        void handleStateChanges(ShooterIntakeSubsystem subsystem) {
            if (this != subsystem.targetState) {
                if (!IDLE.canStart(subsystem)) return;
                this.transitionToState(subsystem, IDLE);
            }
        }

        void onStart(ShooterIntakeSubsystem subsystem) {}

        void onEnd(ShooterIntakeSubsystem subsystem) {}
        
        boolean canStart(ShooterIntakeSubsystem subsystem) {
            return true;
        }

        protected void transitionToState(ShooterIntakeSubsystem subsystem, State newState){
            this.onEnd(subsystem);
            subsystem.currentState = newState;
            newState.onStart(subsystem);
        }
    }
    // spotless:on

}
