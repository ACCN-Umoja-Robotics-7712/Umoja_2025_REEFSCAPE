package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

public class RobotState {

    Elevator elevatorSubSystem;
    CoralArm coralArmSubsystem;
    CoralIntake coralIntakeSubsystem;
    DeepClimb deepClimbSubsystem;
    SwerveSubsystem swerveSubsystem;

    private int state = Constants.RobotStates.NONE;

    public RobotState(SwerveSubsystem swerveSubsystem, Elevator elevatorSubsystem, CoralArm coralArmSubsystem, CoralIntake coralIntakeSubsystem, DeepClimb deepClimbSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.elevatorSubSystem = elevatorSubsystem;
        this.coralArmSubsystem = coralArmSubsystem;
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.deepClimbSubsystem = deepClimbSubsystem;
    }

    public void setState(int state){
        if (this.state != state) {
            this.state = state;
        }
    }

    public int getState(){
        return state;
    }

    public void periodic() {

        // if (state == Constants.RobotStates.PICKUP) {
        //     // if (coralIntakeSubsystem.hasCoral()) {
        //     //     state = Constants.RobotStates.L1;
        //     //     return;
        //     // }
        //     elevatorSubSystem.setState(Constants.ElevatorStates.L1);
        //     coralArmSubsystem.setState(Constants.CoralArmStates.PICKUP);
        //     coralIntakeSubsystem.setState(Constants.CoralIntakeStates.INTAKE);
        //     // algae default position
        // } else if (state == Constants.RobotStates.L1){
        //     // in case we lose coral (robot hits us)
        //     // if (!coralIntakeSubsystem.hasCoral()) {
        //     //     state = Constants.RobotStates.PICKUP;
        //     //     return;
        //     // }
        //     // TODO: if in position switch CORAL INTAKE to SHOOTING state

        //     elevatorSubSystem.setState(Constants.ElevatorStates.L1);
        //     coralArmSubsystem.setState(Constants.CoralArmStates.L1);
        //     coralIntakeSubsystem.setState(Constants.CoralIntakeStates.READY);
        // } else if (state == Constants.RobotStates.L1){
        //     // in case we lose coral (robot hits us)
        //     // if (!coralIntakeSubsystem.hasCoral()) {
        //     //     state = Constants.RobotStates.PICKUP;
        //     //     return;
        //     // }
        //     // TODO: if in position switch CORAL INTAKE to SHOOTING state

        //     elevatorSubSystem.setState(Constants.ElevatorStates.L1);
        //     coralArmSubsystem.setState(Constants.CoralArmStates.L1);
        //     coralIntakeSubsystem.setState(Constants.CoralIntakeStates.READY);
        //     deepClimbSubsystem.setState(Constants.DeepClimbStates.ZERO);
        // } else if (state == Constants.RobotStates.L2){
        //     // in case we lose coral (robot hits us)
        //     // if (!coralIntakeSubsystem.hasCoral()) {
        //     //     state = Constants.RobotStates.PICKUP;
        //     //     return;
        //     // }
        //     // TODO: if in position switch CORAL INTAKE to SHOOTING state

        //     elevatorSubSystem.setState(Constants.ElevatorStates.L2);
        //     coralArmSubsystem.setState(Constants.CoralArmStates.L23);
        //     coralIntakeSubsystem.setState(Constants.CoralIntakeStates.READY);
        //     deepClimbSubsystem.setState(Constants.DeepClimbStates.ZERO);
        // } else if (state == Constants.RobotStates.L3){
        //     // in case we lose coral (robot hits us)
        //     // if (!coralIntakeSubsystem.hasCoral()) {
        //     //     state = Constants.RobotStates.PICKUP;
        //     //     return;
        //     // }
        //     // TODO: if in position switch CORAL INTAKE to SHOOTING state

        //     elevatorSubSystem.setState(Constants.ElevatorStates.L3);
        //     coralArmSubsystem.setState(Constants.CoralArmStates.L23);
        //     coralIntakeSubsystem.setState(Constants.CoralIntakeStates.READY);
        //     deepClimbSubsystem.setState(Constants.DeepClimbStates.ZERO);
        // } else if (state == Constants.RobotStates.L4){
        //     // in case we lose coral (robot hits us)
        //     // if (!coralIntakeSubsystem.hasCoral()) {
        //     //     state = Constants.RobotStates.PICKUP;
        //     //     return;
        //     // }
        //     // TODO: if in position switch CORAL INTAKE to SHOOTING state

        //     elevatorSubSystem.setState(Constants.ElevatorStates.L4);
        //     coralArmSubsystem.setState(Constants.CoralArmStates.L4);
        //     coralIntakeSubsystem.setState(Constants.CoralIntakeStates.READY);
        //     deepClimbSubsystem.setState(Constants.DeepClimbStates.ZERO);
        // } // ADD Algae

        // else if (state == Constants.RobotStates.CLIMB_READY){
        //     deepClimbSubsystem.setState(Constants.DeepClimbStates.READY);
        //     elevatorSubSystem.setState(Constants.ElevatorStates.CLIMB);
        //     if (elevatorSubSystem.isClimbReady()){
        //         coralArmSubsystem.setState(Constants.CoralArmStates.CLIMB);
        //     }
        // }
        // else if (state == Constants.RobotStates.CLIMBING){
        //     deepClimbSubsystem.setState(Constants.DeepClimbStates.CLIMBING);
        // }
        // else if (state == Constants.RobotStates.REMOVE){
        //     //TODO: Finish this
        // }
    }
}
