package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorStates;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.Constants.RobotStates;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.RobotState;

public class OperatorJoystick extends Command {
    //Defining subsystems
    Elevator elevatorSubsystem;
    CoralArm coralArmSubsystem;

    RobotState robotState;

    Joystick j = new Joystick(USB.OPERATOR_CONTROLLER);

    public OperatorJoystick(RobotState robotState, Elevator elevatorSubsystem, CoralArm coralArmSubsystem, Joystick j){
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralArmSubsystem = coralArmSubsystem;

        this.robotState = robotState;

        //Add subsystem dependencies
        addRequirements(elevatorSubsystem, coralArmSubsystem);
    }

    @Override
    public void execute(){

        double elevatorSpeed = Math.abs(j.getRawAxis(OIConstants.LY)) > OIConstants.kDeadband ? j.getRawAxis(OIConstants.LY) * 0.3 : 0.0;
        
        double coralArmSpeed = Math.abs(j.getRawAxis(OIConstants.RY)) > OIConstants.kDeadband ? j.getRawAxis(OIConstants.RY) * 0.3 : 0.0;

        //Elevator Logic --> Switch to coralArm after testing
        // if(j.getRawAxis(OIConstants.RT) > 0.5){
        //     elevatorSubsystem.runElevator(-0.3);
        // } else if(j.getRawAxis(OIConstants.LT) > 0.5) {
        //     elevatorSubsystem.runElevator(0.3);
        //     elevatorSubsystem.setState(ElevatorStates.NONE);
        // } else {
        //     elevatorSubsystem.runElevator(0);
        //     elevatorSubsystem.setState(ElevatorStates.NONE);
        // }

        // elevatorSubsystem.runElevator(elevatorSpeed);
        // coralArmSubsystem.runCoralArm(coralArmSpeed);
    

        if (j.getRawButtonPressed(OIConstants.A)){
            robotState.setState(RobotStates.L1);
        } else if (j.getRawButtonPressed(OIConstants.X)){
            robotState.setState(RobotStates.L2);
        } else if(j.getRawButtonPressed(OIConstants.B)){
            robotState.setState(RobotStates.L3);
        } else if(j.getRawButtonPressed(OIConstants.Y)){
            robotState.setState(RobotStates.L4);
        } else {
            robotState.setState(RobotStates.NONE);
        }

    }
}