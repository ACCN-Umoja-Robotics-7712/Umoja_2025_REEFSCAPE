package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorStates;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.Constants.RobotStates;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DeepClimb;
import frc.robot.subsystems.RobotState;

public class OperatorJoystick extends Command {
    //Defining subsystems
    Elevator elevatorSubsystem;
    CoralArm coralArmSubsystem;
    CoralIntake coralIntakeSubsystem;
    DeepClimb deepClimbSubsystem;

    RobotState robotState;

    Joystick j = new Joystick(USB.OPERATOR_CONTROLLER);

    public OperatorJoystick(RobotState robotState, Elevator elevatorSubsystem, CoralArm coralArmSubsystem, CoralIntake coralIntakeSubsystem, DeepClimb deepClimbSubsystem, Joystick j){
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralArmSubsystem = coralArmSubsystem;
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.deepClimbSubsystem = deepClimbSubsystem;

        this.robotState = robotState;

        //Add subsystem dependencies
        addRequirements(elevatorSubsystem, coralArmSubsystem, coralIntakeSubsystem, deepClimbSubsystem);
    }

    @Override
    public void execute(){

        // manual controls
        double elevatorSpeed = Math.abs(j.getRawAxis(OIConstants.LY)) > OIConstants.kDeadband ? -j.getRawAxis(OIConstants.LY) * 0.9 : 0.0;
        double coralArmSpeed = Math.abs(j.getRawAxis(OIConstants.RY)) > OIConstants.kDeadband ? -j.getRawAxis(OIConstants.RY) * 0.5 : 0.0;
        double climbSpeed = Math.abs(j.getRawAxis(OIConstants.RT)) > OIConstants.kDeadband ? j.getRawAxis(OIConstants.RT) * 0.5 : 0.0;
        double declimbSpeed = Math.abs(j.getRawAxis(OIConstants.LT)) > OIConstants.kDeadband ? -j.getRawAxis(OIConstants.LT) * 0.5 : 0.0;

        if (elevatorSubsystem.getState() == Constants.ElevatorStates.NONE) {
            elevatorSubsystem.runElevator(elevatorSpeed);
        }
        if (coralArmSubsystem.getState() == Constants.CoralArmStates.NONE) {
            coralArmSubsystem.runArm(coralArmSpeed);
        }
        if (climbSpeed != 0) {
            deepClimbSubsystem.runClimber(climbSpeed);
        } else {
            deepClimbSubsystem.runClimber(declimbSpeed);
        }
        
        if (elevatorSpeed != 0){
            elevatorSubsystem.setState(Constants.ElevatorStates.NONE);
        }
        if (coralArmSpeed != 0){
            coralArmSubsystem.setState(Constants.CoralArmStates.NONE);
        }

        boolean intake = j.getRawButton(OIConstants.LB);
        boolean shoot = j.getRawButton(OIConstants.RB);
        if (intake) {
            coralIntakeSubsystem.runIntake(0.4);
        } else if (shoot) {
            coralIntakeSubsystem.runIntake(-0.4);
        } else {
            coralIntakeSubsystem.runIntake(0);
        }

        if (j.getRawButtonPressed(OIConstants.A)){
            elevatorSubsystem.setState(Constants.ElevatorStates.L1);
        }
        if (j.getRawButtonPressed(OIConstants.X)){
            elevatorSubsystem.setState(Constants.ElevatorStates.L2);
        }
        if (j.getRawButtonPressed(OIConstants.B)){
            elevatorSubsystem.setState(Constants.ElevatorStates.L3);
        }
        if (j.getRawButtonPressed(OIConstants.Y)){
            elevatorSubsystem.setState(Constants.ElevatorStates.L4);
        }

        // automated control
        // if (j.getRawButtonPressed(OIConstants.A)){
        //     if (robotState.getState() != RobotStates.CLIMBING || robotState.getState() != RobotStates.CLIMB_READY){
        //         robotState.setState(RobotStates.L1);
        //     }
        // } else if (j.getRawButtonPressed(OIConstants.X)){
        //     robotState.setState(RobotStates.L2);
        // } else if(j.getRawButtonPressed(OIConstants.B)){
        //     robotState.setState(RobotStates.L3);
        // } else if(j.getRawButtonPressed(OIConstants.Y)){
        //     robotState.setState(RobotStates.L4);
        // } else {
        //     robotState.setState(RobotStates.NONE);
        // }
// To-do: Add Climb Ready, Climbing, and remove algae
    }
}