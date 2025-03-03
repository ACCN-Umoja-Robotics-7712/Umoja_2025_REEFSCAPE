package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class Shoot extends Command{
    CoralIntake intake;
    double currentTime, armPosition;

    public Shoot(CoralIntake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if (intake.hasCoralSensor()) {
            intake.runIntake(-1);
        } else {
            intake.runIntake(0);
        }
    }

    @Override
    public void end(boolean isInterrupted){
        // intake.runIntake(0);
    }

    @Override
    public boolean isFinished() {
        return !intake.hasCoralSensor();
    }
}