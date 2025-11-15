package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class MoveArm extends Command{
    CoralArm arm;
    double armState;

    public MoveArm(CoralArm arm, double armState){
        this.arm = arm;
        this.armState = armState;

        addRequirements(arm);
    }

    @Override
    public void initialize(){
        System.out.println("State ARM start");
        arm.setState(armState);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean isInterrupted){
        System.out.println("State ARM END interrupted: " + isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return arm.didReachState();
    }
}