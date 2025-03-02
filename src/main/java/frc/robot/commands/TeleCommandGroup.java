package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PSConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DeepClimb;

public class TeleCommandGroup extends ParallelCommandGroup{
    public TeleCommandGroup(RobotState robotState, SwerveSubsystem swerveSubsystem, Joystick driverController, Joystick operatorController, Elevator elevatorSubsystem, CoralArm coralArmSubsystem, CoralIntake coralIntakeSubsystem, DeepClimb deepClimbSubsystem){
        
        //Driver Command
        addCommands(
            new SwerveJoystick(
                swerveSubsystem,
                () -> -driverController.getRawAxis(PSConstants.LY), 
                () -> -driverController.getRawAxis(PSConstants.LX),
                () -> -driverController.getRawAxis(PSConstants.RX)
            )
        ); // Was DriverYAxis, DriverXAxis, DriverRotAxis

                // operator
        addCommands(
            new OperatorJoystick(robotState, elevatorSubsystem, coralArmSubsystem, coralIntakeSubsystem, deepClimbSubsystem, operatorController)
                );
    }
}

