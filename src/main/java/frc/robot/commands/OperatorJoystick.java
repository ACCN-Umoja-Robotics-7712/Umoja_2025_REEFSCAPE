package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Colors;
import frc.robot.Constants.CoralArmStates;
import frc.robot.Constants.ElevatorStates;
import frc.robot.Constants.GameConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.USB;
import frc.robot.Constants.XBoxConstants;
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

        if (DriverStation.getMatchType() != MatchType.None && DriverStation.getMatchTime() <= 35 && DriverStation.getMatchTime() >= 10) {
            j.setRumble(RumbleType.kBothRumble, 1);
            RobotContainer.led.setLEDColor(Colors.uGold);
        } else {
            j.setRumble(RumbleType.kBothRumble, 0);
        }

        // manual controls
        double elevatorSpeed = Math.abs(j.getRawAxis(XBoxConstants.LY)) > OIConstants.kDeadband ? -j.getRawAxis(XBoxConstants.LY) * 0.8 : 0.0;
        double coralArmSpeed = Math.abs(j.getRawAxis(XBoxConstants.RY)) > OIConstants.kDeadband ? -j.getRawAxis(XBoxConstants.RY) * 0.7 : 0.0;
        double climbSpeed = Math.abs(j.getRawAxis(XBoxConstants.RT)) > OIConstants.kDeadband ? j.getRawAxis(XBoxConstants.RT) * 1 : 0.0;
        double declimbSpeed = Math.abs(j.getRawAxis(XBoxConstants.LT)) > OIConstants.kDeadband ? -j.getRawAxis(XBoxConstants.LT) * 1 : 0.0;

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

        boolean intake = j.getRawButton(XBoxConstants.LB);
        boolean shoot = j.getRawButton(XBoxConstants.R1);
        if (intake) {
            coralIntakeSubsystem.runIntake(1);
        } else if (shoot) {
            if (!(coralArmSubsystem.getState() == CoralArmStates.ALGAE)) {
                coralIntakeSubsystem.runIntake(-1);
            } else {
                coralIntakeSubsystem.runIntake(-0.5);
            }
            // New Addition, W.I.P.
            if (!(coralArmSubsystem.getState() == CoralArmStates.Barge_1)) { //j.getRawButton(XboxConstants.L3)
                coralIntakeSubsystem.runIntake(-1);
            } else {
                coralIntakeSubsystem.runIntake(-1);
            }
            // Flicks arm after shooting on l4 or l1
            if (!coralIntakeSubsystem.hasCoralSensor() && ((coralArmSubsystem.getState() == CoralArmStates.L4 && elevatorSubsystem.getState() == ElevatorStates.L4) || (coralArmSubsystem.getState() == CoralArmStates.L1 && elevatorSubsystem.getState() == ElevatorStates.L1))) {
                coralArmSubsystem.setState(Constants.CoralArmStates.PICKUP);
                elevatorSubsystem.setState(Constants.ElevatorStates.L1);
            }
        } else {
            coralIntakeSubsystem.runIntake(0);
        }

        if (RobotContainer.gameState != GameConstants.Robot){
            if (j.getRawButtonPressed(XBoxConstants.A)){
                coralArmSubsystem.setState(Constants.CoralArmStates.PICKUP);
                elevatorSubsystem.setState(Constants.ElevatorStates.L1);
            }
            if (j.getRawButtonPressed(XBoxConstants.X)){
                elevatorSubsystem.setState(Constants.ElevatorStates.L2);
                coralArmSubsystem.setState(Constants.CoralArmStates.L23);
            }
            if (j.getRawButtonPressed(XBoxConstants.B)){
                elevatorSubsystem.setState(Constants.ElevatorStates.L3);
                coralArmSubsystem.setState(Constants.CoralArmStates.L23);
            }
            if (j.getRawButtonPressed(XBoxConstants.Y)){
                elevatorSubsystem.setState(Constants.ElevatorStates.L4);
                coralArmSubsystem.setState(Constants.CoralArmStates.L4);
            }
            if (j.getRawButtonPressed(XBoxConstants.MENU)){
                elevatorSubsystem.setState(Constants.ElevatorStates.L1);
                coralArmSubsystem.setState(Constants.CoralArmStates.ALGAE);
            }
            if (j.getRawButtonPressed(XBoxConstants.PAGE)){
                elevatorSubsystem.setState(Constants.ElevatorStates.ALGAE);
                coralArmSubsystem.setState(Constants.CoralArmStates.L23);
            }
            // New Addition, W.I.P.
            // if (j.getRawButtonPressed(XBoxConstants.L3)){
            //     coralArmSubsystem.setState(Constants.CoralArmStates.Barge_1);
            //     elevatorSubsystem.setState(Constants.ElevatorStates.L1);
            // }
            // // New Addition, W.I.P.
            // if (j.getRawButtonPressed(XBoxConstants.R3)){
            //     coralArmSubsystem.setState(Constants.CoralArmStates.Barge_2);
                // elevatorSubsystem.setState(Constants.ElevatorStates.BARGE)
                // RobotContainer.led.setLEDColor(uGold);
            // }
        } else {
            if (elevatorSubsystem.getState() != ElevatorStates.NONE) {
                elevatorSubsystem.setState(ElevatorStates.NONE);
            }
            if (coralArmSubsystem.getState() != CoralArmStates.NONE) {
                coralArmSubsystem.setState(CoralArmStates.NONE);
            }
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