package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class TimedShoot extends Command{
    CoralIntake intake;
    double time;


    public TimedShoot(CoralIntake intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize(){
        time = Timer.getTimestamp();
    }

    @Override
    public void execute(){
        intake.runIntake(-1);
    }

    @Override
    public void end(boolean isInterrupted){
        intake.runIntake(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(time-Timer.getTimestamp()) > 3;
    }
}