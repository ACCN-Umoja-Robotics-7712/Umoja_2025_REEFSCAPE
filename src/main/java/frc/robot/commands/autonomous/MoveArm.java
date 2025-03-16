// package frc.robot.commands.autonomous;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CoralArm;

// public class MoveArm extends Command{
//     CoralArm arm;
//     double armState;

//     public MoveArm(CoralArm arm, double armState){
//         this.arm = arm;
//         this.armState = armState;

//         addRequirements(arm);
//     }

//     @Override
//     public void initialize(){
//         arm.setState(armState);
//     }

//     @Override
//     public void execute(){
//     }

//     @Override
//     public void end(boolean isInterrupted){
//         System.out.println("ARM END");
//     }

//     @Override
//     public boolean isFinished() {
//         return arm.didReachState();
//     }
// }