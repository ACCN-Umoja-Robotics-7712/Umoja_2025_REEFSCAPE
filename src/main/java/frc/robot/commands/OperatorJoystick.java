package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorStates;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralArm;

public class OperatorJoystick extends Command {
    //Defining subsystems
    Elevator elevatorSubsystem;
    CoralArm coralArmSubsystem;

    Joystick j = new Joystick(USB.OPERATOR_CONTROLLER);

    public OperatorJoystick(Elevator elevatorSubsystem, CoralArm coralArmSubsystem, Joystick j){
        this.elevatorSubsystem = elevatorSubsystem;

        //Add subsystem dependencies
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute(){

        double elevatorSpeed = Math.abs(j.getRawAxis(OIConstants.LY)) > OIConstants.kDeadband ? j.getRawAxis(OIConstants.LY) * 0.3 : 0.0;
        
        //Elevator Logic
        if(j.getRawAxis(OIConstants.RT) > 0.5){
            elevatorSubsystem.runElevator(-0.3);
        } else if(j.getRawAxis(OIConstants.LT) > 0.5) {
            elevatorSubsystem.runElevator(0.3);
            elevatorSubsystem.setState(ElevatorStates.NONE);
        } else {
            elevatorSubsystem.runElevator(0);
            elevatorSubsystem.setState(ElevatorStates.NONE);
        }

        if (j.getRawButtonPressed(OIConstants.A)){
            elevatorSubsystem.setState(ElevatorStates.L1);
            
        } else if (j.getRawButtonPressed(OIConstants.X)){
            elevatorSubsystem.setState(ElevatorStates.L2);      
        } else if(j.getRawButtonPressed(OIConstants.B)){
            elevatorSubsystem.setState(ElevatorStates.L3);       
        } else if(j.getRawButtonPressed(OIConstants.A)){
            elevatorSubsystem.setState(ElevatorStates.L4);
        } else {
            elevatorSubsystem.setState(ElevatorStates.NONE);
        }

    }
}