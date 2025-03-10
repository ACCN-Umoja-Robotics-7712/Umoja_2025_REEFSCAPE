package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command{
    Elevator elevator;
    double elevatorState;

    public MoveElevator(Elevator elevator, double elevatorState){
        this.elevator = elevator;
        this.elevatorState = elevatorState;

        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        elevator.setState(elevatorState);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("ELEVATOR END");
    }

    @Override
    public boolean isFinished() {
        return elevator.didReachState();
    }
}