package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class ManualArmCommand extends Command{
    CoralArm armSubsystem;
    Supplier<Double> arm_Speed;

    public ManualArmCommand(CoralArm armSubsystem, Supplier<Double> armSpeed){
        this.armSubsystem = armSubsystem;
        this.arm_Speed = armSpeed;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        System.out.println("ARM Initialized");
    }

    @Override
    public void execute(){
        armSubsystem.runArm(arm_Speed.get());
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Arm end is interrupted:" + isInterrupted);
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
