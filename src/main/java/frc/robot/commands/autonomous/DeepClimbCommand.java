package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeepClimb;

public class DeepClimbCommand extends Command{
    DeepClimb deepClimb;
    Supplier<Double> climbSpeed;

    public DeepClimbCommand(DeepClimb deepClimb, Supplier<Double> climbSpeed){
        this.deepClimb = deepClimb;
        this.climbSpeed = climbSpeed;

        addRequirements(deepClimb);
    }

    @Override
    public void initialize(){
        System.out.println("Intake Initialized");
    }

    @Override
    public void execute(){
        deepClimb.runClimber(climbSpeed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Intake end is interrupted:" + isInterrupted);
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
