package frc.robot.subsystems.shooter_intake;

public class ShooterIntakeSubsystem {

    State state = State.IDLE;
    State newState = State.IDLE;
    ShooterIntakeIO io;

    public ShooterIntakeSubsystem(ShooterIntakeIO io){
        this.io = io;
    }

    public void periodic(){

        state.periodic(this);

    }

    public void setNewState(State newState){
        this.newState = newState;
    }

    private enum State{
        IDLE{
            @Override
            void changeStates(ShooterIntakeSubsystem subsystem){
                subsystem.state = subsystem.newState;
            }

            @Override
            void periodic(ShooterIntakeSubsystem subsystem){
                super.periodic(subsystem);
                subsystem.io.setVoltage(0);
            }
        },

        INTAKING{
            //TODO add periodic for intake
        },

        SHOOTING{
            //TODO add periodic for shooting
        };

        void periodic(ShooterIntakeSubsystem subsystem){
            this.changeStates(subsystem);
        }

        void changeStates(ShooterIntakeSubsystem subsystem){
            subsystem.state = subsystem.newState;
        }
    }

}
