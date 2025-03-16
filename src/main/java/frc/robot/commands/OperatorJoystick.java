package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorStates;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.Constants.XBoxConstants;
import frc.robot.Constants.RobotStates;
import frc.robot.subsystems.CoralIntake;

public class OperatorJoystick extends Command {
    //Defining subsystems
    CoralIntake coralIntakeSubsystem;

    Joystick j = new Joystick(USB.OPERATOR_CONTROLLER);

    public OperatorJoystick(CoralIntake coralIntakeSubsystem, Joystick j){
        this.coralIntakeSubsystem = coralIntakeSubsystem;

        //Add subsystem dependencies
        addRequirements(coralIntakeSubsystem);
    }

    @Override
    public void execute(){

        // if (DriverStation.getMatchType() != MatchType.None && DriverStation.getMatchTime() <= 30 && DriverStation.getMatchTime() >= 20) {
        //     j.setRumble(RumbleType.kBothRumble, 1);
        // } else {
        //     j.setRumble(RumbleType.kBothRumble, 0);
        // }

       

        boolean intake = j.getRawButton(XBoxConstants.LB);
        boolean shoot = j.getRawButton(XBoxConstants.R1);

        if (intake) {
            coralIntakeSubsystem.runIntake(0.8);
        } else if (shoot) {
            coralIntakeSubsystem.runIntake(-0.8);
        } else {
            coralIntakeSubsystem.runIntake(0);
        }

        // automated control
        // if (j.getRawButtonPressed(XBoxConstants.A)){
        //     if (robotState.getState() != RobotStates.CLIMBING || robotState.getState() != RobotStates.CLIMB_READY){
        //         robotState.setState(RobotStates.L1);
        //     }
        // } else if (j.getRawButtonPressed(XBoxConstants.X)){
        //     robotState.setState(RobotStates.L2);
        // } else if(j.getRawButtonPressed(XBoxConstants.B)){
        //     robotState.setState(RobotStates.L3);
        // } else if(j.getRawButtonPressed(XBoxConstants.Y)){
        //     robotState.setState(RobotStates.L4);
        // } else {
        //     robotState.setState(RobotStates.NONE);
        // }
// TODO: Add Climb Ready, Climbing, and remove algae
    }
}