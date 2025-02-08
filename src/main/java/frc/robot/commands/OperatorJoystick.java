package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.subsystems.Elevator;

public class OperatorJoystick extends Command {
    //Defining subsystems
    Elevator elevatorSubsystem;

    Joystick j = new Joystick(USB.OPERATOR_CONTROLLER);

    public OperatorJoystick(Elevator elevatorSubsystem, Joystick j){
        this.elevatorSubsystem = elevatorSubsystem;

        //Add subsystem dependencies
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute(){

        //Climber Logic
        if(j.getRawAxis(OIConstants.RT) > 0.5){
            elevatorSubsystem.runElevator(-0.3);
        } else if(j.getRawAxis(OIConstants.LT) > 0.5) {
            elevatorSubsystem.runElevator(0.3);
        } else {
            elevatorSubsystem.runElevator(0);
        }
    }
}