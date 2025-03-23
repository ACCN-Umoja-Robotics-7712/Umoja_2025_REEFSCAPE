package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class Intake extends Command{
    CoralIntake intake;

    public Intake(CoralIntake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize(){
        System.out.println("Initialized");
    }

    @Override
    public void execute(){
        intake.runIntake(1);
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("Is interrupted:" + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return intake.hasCoralSensor();
    }
    
}
