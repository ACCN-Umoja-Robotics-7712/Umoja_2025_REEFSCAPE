package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeepClimb;
import frc.robot.subsystems.Elevator;

public class ManualElevatorCommand extends Command{
    Elevator elevatorSubsystem;
    Supplier<Double> elevator_Speed;

    public ManualElevatorCommand(Elevator elevatorSubsystem, Supplier<Double> elevatorSpeed){
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevator_Speed = elevatorSpeed;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("Elevator Initialized");
    }

    @Override
    public void execute(){
        elevatorSubsystem.runElevator(elevator_Speed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Elevator end is interrupted:" + isInterrupted);
    }

    // @Override
    // public boolean isFinished() {
    //     return deepClimb.hasCoralSensor();
    // }
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
