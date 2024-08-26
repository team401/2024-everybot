package frc.robot.subsystems.shooter_intake;

public class ShooterIntakeSubsystem {

    protected State currentState = State.IDLE;
    protected State targetState = State.IDLE;
    ShooterIntakeIO io;

    public ShooterIntakeSubsystem(ShooterIntakeIO io){
        this.io = io;
        State.subsystem = this;
    }

    public void periodic(){

        currentState.periodic();

    }

    public void setTargetState(State target){
        this.targetState = target;
    }

    public enum State{
        IDLE{
            @Override
            void changeStates(){
                subsystem.currentState = subsystem.targetState;
            }

            @Override
            void onStart(){
                subsystem.io.setVoltage(0);
            }
        },

        INTAKING{
            @Override
            void periodic(){
                super.periodic();
                subsystem.io.setVoltage(-1);
            }
        },

        SHOOTING{
            @Override
            void periodic(){
                super.periodic();
                subsystem.io.setVoltage(1);
            }
        };

        static ShooterIntakeSubsystem subsystem;

        void periodic(){
            this.changeStates();
        }

        void changeStates(){
            if (subsystem.currentState != subsystem.targetState){
                subsystem.currentState = State.IDLE;
            }
        }

        void onStart() {};
        void onEnd() {};
    }

}
